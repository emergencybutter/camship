//! The SimConnect side: connects (and reconnects) to MSFS, keeps the latest
//! aircraft state, and while Camship has the camera, runs the director once
//! per visual frame and sends the pose.

use std::collections::VecDeque;
use std::fs::OpenOptions;
use std::io::{self, Write};
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use camship_core::{CameraPose, Director, Subject};

use crate::config::{Config, PositionMode};
use crate::geo::{self, LocalFrame, RECENTER_M};
use crate::sim::{self, Sample, VARS};
use crate::simconnect::{self as sc, AnyTransport, CameraData, Client, Message};

const APP_NAME: &str = "Camship";
/// Used when the aircraft doesn't report its wingspan, meters.
const DEFAULT_SIZE_M: f64 = 15.0;
/// Without a camera status message this long after acquiring, start anyway.
const ACQUIRE_WAIT: Duration = Duration::from_secs(1);
/// This many camera errors in a row means MSFS won't take our camera.
const MAX_CAMERA_ERRORS: u32 = 120;
const LOG_LINES: usize = 200;

const DEF_AIRCRAFT: u32 = 1;
const REQ_AIRCRAFT: u32 = 1;
const EV_FRAME: u32 = 1;
const EV_PAUSE: u32 = 2;
const EV_AIRCRAFT_LOADED: u32 = 3;
const EV_SIM_START: u32 = 4;
const EV_SIM_STOP: u32 = 5;

pub enum Command {
    Toggle,
    Next,
    SetHold(bool),
    Play(usize),
    Quit,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraState {
    Off,
    Acquiring,
    Running,
}

/// What the window shows and edits.
pub struct Shared {
    pub director: Director,
    pub connected: bool,
    pub status: String,
    pub camera: CameraState,
    pub size: f64,
    pub aircraft: String,
    pub log: VecDeque<String>,
    log_path: PathBuf,
}

impl Shared {
    pub fn new(director: Director, log_path: PathBuf) -> Self {
        let _ = std::fs::write(&log_path, "");
        Self {
            director,
            connected: false,
            status: String::new(),
            camera: CameraState::Off,
            size: DEFAULT_SIZE_M,
            aircraft: "Aircraft: waiting for MSFS".into(),
            log: VecDeque::new(),
            log_path,
        }
    }

    pub fn log(&mut self, line: &str) {
        let stamped = format!("[{}] {line}", clock());
        if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(&self.log_path) {
            let _ = writeln!(f, "{stamped}");
        }
        self.log.push_back(stamped);
        while self.log.len() > LOG_LINES {
            self.log.pop_front();
        }
    }
}

/// Seconds since the app started, for log lines.
fn clock() -> String {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    format!("{:8.2}", START.get_or_init(Instant::now).elapsed().as_secs_f64())
}

fn connect(cfg: &Config) -> io::Result<AnyTransport> {
    if !cfg.address.is_empty() {
        return sc::Tcp::connect(&cfg.address).map(AnyTransport::Tcp);
    }
    #[cfg(windows)]
    {
        sc::Pipe::connect().map(AnyTransport::Pipe)
    }
    #[cfg(not(windows))]
    {
        Err(io::Error::new(io::ErrorKind::Unsupported, "set `address host:port` in camship-msfs.ini"))
    }
}

pub fn run(shared: Arc<Mutex<Shared>>, rx: Receiver<Command>, cfg: Config, repaint: impl Fn()) {
    let lock = || shared.lock().unwrap_or_else(|e| e.into_inner());
    let mut last_error = String::new();
    loop {
        match connect(&cfg) {
            Ok(t) => {
                last_error.clear();
                let mut session = Session::new(&shared, &cfg, &repaint);
                let quit = match session.run(t, &rx) {
                    Ok(quit) => quit,
                    Err(e) => {
                        session.with(|s| s.log(&format!("connection lost: {e}")));
                        false
                    }
                };
                {
                    let mut s = lock();
                    s.connected = false;
                    s.camera = CameraState::Off;
                }
                if quit {
                    return;
                }
            }
            Err(e) => {
                let msg = e.to_string();
                let mut s = lock();
                s.status = "Waiting for MSFS 2024...".into();
                if msg != last_error {
                    s.log(&format!("cannot connect yet: {msg}"));
                    last_error = msg;
                }
            }
        }
        repaint();
        // Wait a bit before retrying, still answering the window.
        let until = Instant::now() + Duration::from_secs(2);
        while Instant::now() < until {
            match rx.try_recv() {
                Ok(Command::Quit) | Err(TryRecvError::Disconnected) => return,
                Ok(Command::Toggle) => lock().log("not connected to MSFS yet"),
                Ok(Command::SetHold(h)) => lock().director.set_hold(h),
                Ok(_) => {}
                Err(TryRecvError::Empty) => std::thread::sleep(Duration::from_millis(50)),
            }
        }
    }
}

struct Session<'a, R: Fn()> {
    shared: &'a Arc<Mutex<Shared>>,
    cfg: &'a Config,
    repaint: &'a R,
    /// Send id of each AddToDataDefinition, to spot rejected variables.
    def_send_ids: Vec<u32>,
    accepted: Vec<bool>,
    sample: Option<Sample>,
    frame: Option<LocalFrame>,
    paused: bool,
    acquire: Option<(u32, Instant)>,
    last_time: Option<Instant>,
    camera_errors: u32,
    span: Option<f64>,
}

impl<'a, R: Fn()> Session<'a, R> {
    fn new(shared: &'a Arc<Mutex<Shared>>, cfg: &'a Config, repaint: &'a R) -> Self {
        Self {
            shared,
            cfg,
            repaint,
            def_send_ids: Vec::new(),
            accepted: vec![true; VARS.len()],
            sample: None,
            frame: None,
            paused: false,
            acquire: None,
            last_time: None,
            camera_errors: 0,
            span: None,
        }
    }

    fn with<T>(&self, f: impl FnOnce(&mut Shared) -> T) -> T {
        f(&mut self.shared.lock().unwrap_or_else(|e| e.into_inner()))
    }

    /// Runs until the connection drops (`Err`) or the app quits (`Ok(true)`).
    fn run(&mut self, t: AnyTransport, rx: &Receiver<Command>) -> io::Result<bool> {
        let mut c = Client::open(t, APP_NAME)?;
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            match c.poll()? {
                Some(Message::Open { sim_version }) => {
                    let (a, b, cc, d) = sim_version;
                    self.with(|s| {
                        s.connected = true;
                        s.status = "Connected to MSFS".into();
                        s.log(&format!("connected to MSFS (SimConnect {a}.{b}.{cc}.{d})"));
                    });
                    break;
                }
                Some(_) => {}
                None if Instant::now() > deadline => {
                    return Err(io::Error::new(io::ErrorKind::TimedOut, "MSFS did not answer the handshake"))
                }
                None => std::thread::sleep(Duration::from_millis(10)),
            }
        }
        for v in VARS {
            self.def_send_ids.push(c.add_to_data_definition(DEF_AIRCRAFT, v.name, v.units)?);
        }
        c.request_data_on_sim_object(REQ_AIRCRAFT, DEF_AIRCRAFT, sc::PERIOD_SIM_FRAME)?;
        c.subscribe_to_system_event(EV_FRAME, "Frame")?;
        c.subscribe_to_system_event(EV_PAUSE, "Pause")?;
        c.subscribe_to_system_event(EV_AIRCRAFT_LOADED, "AircraftLoaded")?;
        c.subscribe_to_system_event(EV_SIM_START, "SimStart")?;
        c.subscribe_to_system_event(EV_SIM_STOP, "SimStop")?;
        c.subscribe_to_camera_status()?;
        (self.repaint)();

        loop {
            let mut busy = false;
            while let Some(msg) = c.poll()? {
                busy = true;
                if self.handle(&mut c, msg)? {
                    return Ok(false);
                }
            }
            loop {
                match rx.try_recv() {
                    Ok(Command::Quit) | Err(TryRecvError::Disconnected) => {
                        if self.camera() != CameraState::Off {
                            self.stop(&mut c, "Camship closed")?;
                        }
                        return Ok(true);
                    }
                    Ok(cmd) => {
                        busy = true;
                        self.command(&mut c, cmd)?;
                    }
                    Err(TryRecvError::Empty) => break,
                }
            }
            if self.camera() == CameraState::Acquiring && self.acquire.is_some_and(|(_, t)| t.elapsed() > ACQUIRE_WAIT) {
                self.with(|s| {
                    s.camera = CameraState::Running;
                    s.log("no camera status from MSFS; assuming the camera is ours");
                });
            }
            if !busy {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    }

    fn camera(&self) -> CameraState {
        self.with(|s| s.camera)
    }

    /// Returns true when the simulator is quitting.
    fn handle(&mut self, c: &mut Client<AnyTransport>, msg: Message) -> io::Result<bool> {
        match msg {
            Message::Quit => {
                self.with(|s| s.log("MSFS is closing"));
                return Ok(true);
            }
            Message::Exception { exception, send_id, index } => {
                if let Some(i) = self.def_send_ids.iter().position(|&id| id == send_id) {
                    self.accepted[i] = false;
                    self.sample = None;
                    self.with(|s| s.log(&format!("MSFS doesn't know {} (exception {exception}); doing without it", VARS[i].name)));
                } else if self.acquire.is_some_and(|(id, _)| id == send_id) {
                    self.acquire = None;
                    self.with(|s| {
                        s.camera = CameraState::Off;
                        s.log(&format!("MSFS refused the camera (exception {exception}); is another add-on using it?"));
                    });
                } else if exception == sc::EXCEPTION_CAMERA_API {
                    self.camera_errors += 1;
                    if self.camera_errors == 1 || self.camera_errors >= MAX_CAMERA_ERRORS {
                        self.with(|s| s.log(&format!("camera error (exception {exception}, parameter {index})")));
                    }
                    if self.camera_errors >= MAX_CAMERA_ERRORS {
                        self.stop(c, "MSFS keeps rejecting the camera")?;
                    }
                } else {
                    self.with(|s| s.log(&format!("SimConnect exception {exception} (packet {send_id}, parameter {index})")));
                }
            }
            Message::SimObjectData { request_id: REQ_AIRCRAFT, values } => {
                self.sample = Sample::from_values(&values, &self.accepted);
                self.refresh_size();
            }
            Message::Event { id: EV_PAUSE, data } => self.paused = data != 0,
            Message::Event { id: EV_SIM_STOP, .. } => {
                if self.camera() != CameraState::Off {
                    self.stop(c, "the flight ended")?;
                }
            }
            Message::Event { id: EV_SIM_START, .. } | Message::EventFilename { id: EV_AIRCRAFT_LOADED, .. } => {
                self.span = None;
            }
            Message::Frame { id: EV_FRAME } => {
                if self.camera() == CameraState::Running {
                    self.camera_frame(c)?;
                }
            }
            Message::CameraStatus { acquired, game_controlled } => {
                let state = self.camera();
                if acquired == sc::CAMERA_ACQUIRED {
                    if state == CameraState::Acquiring {
                        self.with(|s| {
                            s.camera = CameraState::Running;
                            s.log(&format!("camera acquired{}", if game_controlled { " (still under sim control)" } else { "" }));
                        });
                    }
                } else if state != CameraState::Off {
                    let why = match acquired {
                        2 => "another add-on took the camera",
                        3 => "the camera was taken back (view change)",
                        _ => "the camera was released",
                    };
                    self.acquire = None;
                    self.with(|s| {
                        s.camera = CameraState::Off;
                        s.log(&format!("{why} (status {acquired})"));
                    });
                }
            }
            _ => {}
        }
        Ok(false)
    }

    fn command(&mut self, c: &mut Client<AnyTransport>, cmd: Command) -> io::Result<()> {
        match cmd {
            Command::Toggle => match self.camera() {
                CameraState::Off => self.start(c)?,
                _ => self.stop(c, "stopped")?,
            },
            Command::Next => match self.camera() {
                CameraState::Off => self.start(c)?,
                _ => self.with(|s| s.director.next_shot()),
            },
            Command::SetHold(h) => self.with(|s| s.director.set_hold(h)),
            Command::Play(i) => self.with(|s| {
                if s.camera != CameraState::Off {
                    s.director.play(i);
                }
            }),
            Command::Quit => {}
        }
        Ok(())
    }

    fn start(&mut self, c: &mut Client<AnyTransport>) -> io::Result<()> {
        let id = c.camera_acquire(APP_NAME)?;
        c.camera_enable_flag(sc::CAMERA_FLAG_ABOVE_GROUND)?;
        self.acquire = Some((id, Instant::now()));
        self.last_time = None;
        self.camera_errors = 0;
        self.frame = None;
        self.with(|s| {
            s.director.reset();
            s.camera = CameraState::Acquiring;
            s.log("acquiring the camera");
        });
        Ok(())
    }

    fn stop(&mut self, c: &mut Client<AnyTransport>, why: &str) -> io::Result<()> {
        self.acquire = None;
        self.with(|s| {
            s.camera = CameraState::Off;
            s.log(&format!("camera released: {why}"));
        });
        c.camera_release(&self.cfg.release_camera)?;
        Ok(())
    }

    fn refresh_size(&mut self) {
        let Some(sample) = &self.sample else { return };
        let span = sample.wing_span();
        if span == self.span && self.span.is_some() {
            return;
        }
        self.span = span;
        self.with(|s| {
            let configured = s.director.settings().subject_size_m;
            s.size = if configured > 0.0 { configured } else { span.map_or(DEFAULT_SIZE_M, |w| w.clamp(6.0, 90.0)) };
            s.aircraft = format!(
                "This aircraft: {:.0} m across · engine and wheel positions aren't available in MSFS",
                s.size
            );
            s.log(&format!(
                "subject size {:.1} m ({}; set subject_size_m in camship.prf to override)",
                s.size,
                span.map_or("no wingspan reported".to_string(), |w| format!("wingspan {w:.1} m"))
            ));
        });
    }

    fn camera_frame(&mut self, c: &mut Client<AnyTransport>) -> io::Result<()> {
        let Some(sample) = self.sample.clone() else { return Ok(()) };
        let Some((lat, lon, _)) = sample.position() else { return Ok(()) };
        let now = Instant::now();
        let mut dt = self.last_time.map_or(0.0, |t| now.duration_since(t).as_secs_f64()).min(0.25);
        self.last_time = Some(now);

        // Keep the local frame centred near the aircraft.
        let frame = *self.frame.get_or_insert(LocalFrame::new(lat, lon));
        let here = frame.to_local(lat, lon, 0.0);
        let frame = if here.horizontal().len() > RECENTER_M {
            let next = LocalFrame::new(lat, lon);
            self.with(|s| s.director.shift_origin(here));
            self.frame = Some(next);
            next
        } else {
            frame
        };

        let ground_alt = sample.ground_altitude();
        let sun = sample.sun();
        let cfg = self.cfg;
        let pose_and_subject = self.with(|s| -> Option<(CameraPose, Subject)> {
            if self.paused && !s.director.settings().film_when_paused {
                dt = 0.0;
            }
            let subject = sample.subject(&frame, s.size)?;
            if let Some((az, el)) = sun {
                s.director.set_sun(az, el);
            }
            s.director.set_fov(cfg.fov_deg);
            // MSFS only tells us the ground height under the aircraft.
            let mut ground = |_p| ground_alt;
            let pose = s.director.update(&subject, dt, &mut ground);
            for note in s.director.take_notes() {
                s.log(&note);
            }
            Some((pose, subject))
        });
        let Some((pose, subject)) = pose_and_subject else { return Ok(()) };
        let data = camera_data(cfg, &frame, &subject, &pose);
        let mask = sc::CAMERA_MASK_POSITION | sc::CAMERA_MASK_ROTATION | sc::CAMERA_MASK_FOV | sc::CAMERA_MASK_REFERENTIAL;
        c.camera_set(&data, mask)?;
        Ok(())
    }
}

/// The director's pose in the terms of MSFS's camera API.
pub fn camera_data(cfg: &Config, frame: &LocalFrame, subject: &Subject, pose: &CameraPose) -> CameraData {
    let (position, position_referential) = match cfg.position {
        PositionMode::Aircraft => {
            (geo::body_to_msfs(sim::body_offset(subject, pose.pos)), sc::POSITION_REFERENTIAL_SIMOBJECT)
        }
        PositionMode::World => {
            let (lat, lon, alt) = frame.to_world(pose.pos);
            ([lat, lon, alt], sc::POSITION_REFERENTIAL_WORLD)
        }
    };
    let fov_deg = geo::zoomed_fov(cfg.fov_deg, pose.zoom);
    CameraData {
        position,
        position_referential,
        position_object: sc::OBJECT_ID_USER,
        target: [0.0; 3],
        pbh: [
            (cfg.pitch_sign * pose.pitch) as f32,
            (cfg.bank_sign * pose.roll) as f32,
            pose.heading.rem_euclid(360.0) as f32,
        ],
        rotation_referential: sc::POSITION_REFERENTIAL_WORLD,
        rotation_object: sc::OBJECT_ID_USER,
        fov: if cfg.fov_radians { fov_deg.to_radians() } else { fov_deg },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use camship_core::{Attitude, Systems, Vec3};

    /// A fake simulator on TCP: answers the handshake, streams aircraft data
    /// and frames, and records the function id of every packet it receives.
    #[test]
    fn films_against_a_fake_simulator() {
        use std::io::Read;
        use std::net::TcpListener;
        use std::sync::mpsc;

        fn msg(id: u32, body: &[u8]) -> Vec<u8> {
            let mut m = Vec::new();
            m.extend_from_slice(&((12 + body.len()) as u32).to_le_bytes());
            m.extend_from_slice(&sc::PROTOCOL.to_le_bytes());
            m.extend_from_slice(&id.to_le_bytes());
            m.extend_from_slice(body);
            m
        }
        fn words(w: &[u32]) -> Vec<u8> {
            w.iter().flat_map(|x| x.to_le_bytes()).collect()
        }

        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let dir = std::env::temp_dir().join(format!("camship-msfs-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let shared = Arc::new(Mutex::new(Shared::new(Director::new(Default::default(), 7), dir.join("Log.txt"))));
        let (tx, rx) = mpsc::channel();
        let cfg = Config { address: addr, ..Config::default() };
        let worker_shared = shared.clone();
        let worker = std::thread::spawn(move || run(worker_shared, rx, cfg, || {}));

        let (mut sock, _) = listener.accept().unwrap();
        sock.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        let mut functions = Vec::new();
        let mut inbox = Vec::new();
        let mut read = |sock: &mut std::net::TcpStream, functions: &mut Vec<u32>| {
            let mut buf = [0u8; 65536];
            if let Ok(n) = sock.read(&mut buf) {
                inbox.extend_from_slice(&buf[..n]);
            }
            while inbox.len() >= 16 {
                let size = u32::from_le_bytes(inbox[0..4].try_into().unwrap()) as usize;
                if inbox.len() < size {
                    break;
                }
                functions.push(u32::from_le_bytes(inbox[8..12].try_into().unwrap()) & 0xFFFF);
                inbox.drain(..size);
            }
        };

        read(&mut sock, &mut functions);
        assert_eq!(functions.first(), Some(&0x01), "handshake first");
        let mut open = vec![0u8; 256 + 16];
        open.extend(words(&[12, 4, 0, 0, 0, 0]));
        sock.write_all(&msg(2, &open)).unwrap();

        // Aircraft data: 45°N 7°E, 1000 m, flying east at 60 m/s.
        let mut values = vec![0.0f64; VARS.len()];
        values[0] = 45.0;
        values[1] = 7.0;
        values[2] = 1000.0;
        values[3] = 800.0;
        values[4] = 200.0;
        values[6] = 90.0;
        values[9] = 60.0;
        values[12] = 30.0;
        values[13] = 1.0;
        let mut data = words(&[REQ_AIRCRAFT, 0, DEF_AIRCRAFT, 0, 0, 1, VARS.len() as u32]);
        data.extend(values.iter().flat_map(|v| v.to_le_bytes()));
        let frame = msg(7, &[words(&[0, EV_FRAME, 0]), 60f32.to_le_bytes().to_vec(), 1f32.to_le_bytes().to_vec()].concat());

        let deadline = Instant::now() + Duration::from_secs(5);
        let mut toggled = false;
        while Instant::now() < deadline {
            sock.write_all(&msg(8, &data)).unwrap();
            sock.write_all(&frame).unwrap();
            read(&mut sock, &mut functions);
            if !toggled && shared.lock().unwrap().connected {
                tx.send(Command::Toggle).unwrap();
                toggled = true;
            }
            if toggled && functions.contains(&0x5f) {
                // Camera status: acquired.
                sock.write_all(&msg(41, &words(&[1, 0]))).unwrap();
            }
            if functions.iter().filter(|&&f| f == 0x62).count() >= 20 {
                break;
            }
        }
        assert!(functions.contains(&0x0c) && functions.contains(&0x0e), "data definition and request: {functions:x?}");
        assert!(functions.iter().filter(|&&f| f == 0x62).count() >= 20, "camera updates: {functions:x?}");
        assert_eq!(shared.lock().unwrap().camera, CameraState::Running);
        assert!(shared.lock().unwrap().director.current_shot().is_some());
        assert!((shared.lock().unwrap().size - 30.0).abs() < 1e-9);

        tx.send(Command::Quit).unwrap();
        worker.join().unwrap();
        read(&mut sock, &mut functions);
        assert_eq!(functions.last(), Some(&0x60), "camera released on quit");
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn camera_behind_a_north_bound_aircraft() {
        let subject = Subject {
            pos: Vec3::new(0.0, 100.0, 0.0),
            vel: Vec3::ZERO,
            att: Attitude::new(0.0, 0.0, 0.0),
            agl: 100.0,
            on_ground: false,
            size: 30.0,
            systems: Systems::default(),
        };
        // 50 m south (behind), 5 m up, looking north and 10° down.
        let pose = CameraPose { pos: Vec3::new(0.0, 105.0, 50.0), heading: 0.0, pitch: -10.0, roll: 5.0, zoom: 1.0 };
        let cfg = Config::default();
        let d = camera_data(&cfg, &LocalFrame::new(0.0, 0.0), &subject, &pose);
        assert_eq!(d.position_referential, sc::POSITION_REFERENTIAL_SIMOBJECT);
        let [x, y, z] = d.position;
        assert!(x.abs() < 1e-9 && (y - 5.0).abs() < 1e-9 && (z + 50.0).abs() < 1e-9, "{:?}", d.position);
        assert_eq!(d.pbh, [10.0, -5.0, 0.0]);
        assert!((d.fov - 60f64.to_radians()).abs() < 1e-9);

        let world = Config { position: PositionMode::World, ..Config::default() };
        let d = camera_data(&world, &LocalFrame::new(0.0, 0.0), &subject, &pose);
        assert!(d.position[0] < 0.0 && d.position[1].abs() < 1e-12 && (d.position[2] - 105.0).abs() < 1e-9);
    }
}
