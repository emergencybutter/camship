//! Plugin state: engaging/releasing X-Plane's camera, feeding the director,
//! the settings window, the menu and preferences.

use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use camship_core::{Attitude, CameraPose, Director, Settings, Subject, Systems, Vec3};

use crate::gui::EguiWindow;
use crate::view::{self, Action, ViewModel};
use crate::xp::{self, DataRef, Menu, Probe};

pub const MENU_TOGGLE: usize = 1;
pub const MENU_NEXT: usize = 2;
pub const MENU_SETTINGS: usize = 3;

/// Settings edited in the window are written to disk once they settle.
const SAVE_DELAY: Duration = Duration::from_millis(800);

/// Losing the camera this soon after a screenshot isn't the user leaving: take it back.
const CAPTURE_GRACE_S: f64 = 2.0;
/// Used when the aircraft doesn't report its size, meters.
const DEFAULT_SIZE_M: f64 = 15.0;
/// A view command this recent explains losing the camera: the user picked a view.
const VIEW_CMD_WINDOW: Duration = Duration::from_millis(500);
/// Losing the camera with no view command (another plugin taking it, e.g. the
/// DWSim ATR's SASL code) is not the user leaving, so we take it back. More than
/// this many times in `RETAKE_WINDOW` is a fight we won't win: give up.
const MAX_RETAKES: usize = 3;
const RETAKE_WINDOW: Duration = Duration::from_secs(60);

/// X-Plane commands that change the view. Watched (never swallowed) so that
/// losing the camera can be told apart: right after one of these it's the
/// user's choice; without one, someone else took the camera.
pub const VIEW_COMMANDS: &[&str] = &[
    "sim/view/ai_controls_views",
    "sim/view/free_camera",
    "sim/view/default_view",
    "sim/view/forward_with_2d_panel",
    "sim/view/forward_with_hud",
    "sim/view/forward_with_nothing",
    "sim/view/linear_spot",
    "sim/view/still_spot",
    "sim/view/runway",
    "sim/view/circle",
    "sim/view/tower",
    "sim/view/ridealong",
    "sim/view/track_weapon",
    "sim/view/chase",
    "sim/view/wingman",
    "sim/view/3d_cockpit_cmnd_look",
    "sim/view/3d_cockpit_toggle",
    "sim/view/cinema_verite",
    "sim/view/glance_left",
    "sim/view/glance_right",
    "sim/view/up_left",
    "sim/view/up_right",
    "sim/view/straight_up",
    "sim/view/straight_down",
    "sim/view/left_45",
    "sim/view/right_45",
    "sim/view/left_90",
    "sim/view/right_90",
    "sim/view/left_135",
    "sim/view/right_135",
    "sim/view/back",
    "sim/view/quick_look_0",
    "sim/view/quick_look_1",
    "sim/view/quick_look_2",
    "sim/view/quick_look_3",
    "sim/view/quick_look_4",
    "sim/view/quick_look_5",
    "sim/view/quick_look_6",
    "sim/view/quick_look_7",
    "sim/view/quick_look_8",
    "sim/view/quick_look_9",
    "sim/view/quick_look_10",
    "sim/view/quick_look_11",
    "sim/view/quick_look_12",
    "sim/view/quick_look_13",
    "sim/view/quick_look_14",
    "sim/view/quick_look_15",
    "sim/view/quick_look_16",
    "sim/view/quick_look_17",
    "sim/view/quick_look_18",
    "sim/view/quick_look_19",
];

#[derive(Clone, Copy, Debug)]
pub enum Event {
    Toggle,
    NextShot,
    ToggleHold,
    SetHold(bool),
    /// Cut to shot `i` now.
    Play(usize),
    ToggleWindow,
    /// A screenshot or movie command ran.
    Capture,
    /// The view switch to an outside view has happened; take the camera.
    TakeControl,
    /// X-Plane took the camera back (the user picked another view, or
    /// another plugin took it).
    LostControl,
    /// `VIEW_COMMANDS[i]` is about to run.
    ViewCommand(usize),
    NewFlight,
}

/// What `App` asks of `lib.rs`, which owns the C callbacks.
pub enum Request {
    None,
    /// Schedule the one-shot flight loop that sends `TakeControl` next frame.
    TakeControlNextFrame,
    /// Hand our camera callback to `XPLMControlCamera`.
    ControlCamera,
    /// Call `XPLMDontControlCamera`.
    ReleaseCamera,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Off,
    /// Switched to an outside view, waiting one frame before taking control.
    Starting,
    Running,
}

struct Refs {
    x: Option<DataRef>,
    y: Option<DataRef>,
    z: Option<DataRef>,
    vx: Option<DataRef>,
    vy: Option<DataRef>,
    vz: Option<DataRef>,
    psi: Option<DataRef>,
    theta: Option<DataRef>,
    phi: Option<DataRef>,
    agl: Option<DataRef>,
    on_ground: Option<DataRef>,
    size_x: Option<DataRef>,
    size_z: Option<DataRef>,
    view_type: Option<DataRef>,
    num_engines: Option<DataRef>,
    engine_x: Option<DataRef>,
    engine_y: Option<DataRef>,
    engine_z: Option<DataRef>,
    sun_heading: Option<DataRef>,
    sun_pitch: Option<DataRef>,
    fov: Option<DataRef>,
    paused: Option<DataRef>,
    gear_deploy: Option<DataRef>,
    flaps: Option<DataRef>,
    reversers: Option<DataRef>,
    speedbrake: Option<DataRef>,
    landing_lights: Option<DataRef>,
    landing_light_switch: Option<DataRef>,
    doors: Option<DataRef>,
    canopy: Option<DataRef>,
    engines_burning: Option<DataRef>,
    /// Wheel positions, body frame, meters.
    wheel_x: Option<DataRef>,
    wheel_y: Option<DataRef>,
    wheel_z: Option<DataRef>,
}

impl Refs {
    fn find() -> Self {
        let f = DataRef::find;
        Self {
            x: f("sim/flightmodel/position/local_x"),
            y: f("sim/flightmodel/position/local_y"),
            z: f("sim/flightmodel/position/local_z"),
            vx: f("sim/flightmodel/position/local_vx"),
            vy: f("sim/flightmodel/position/local_vy"),
            vz: f("sim/flightmodel/position/local_vz"),
            psi: f("sim/flightmodel/position/psi"),
            theta: f("sim/flightmodel/position/theta"),
            phi: f("sim/flightmodel/position/phi"),
            agl: f("sim/flightmodel/position/y_agl"),
            on_ground: f("sim/flightmodel/failures/onground_any"),
            size_x: f("sim/aircraft/view/acf_size_x"),
            size_z: f("sim/aircraft/view/acf_size_z"),
            view_type: f("sim/graphics/view/view_type"),
            num_engines: f("sim/aircraft/engine/acf_num_engines"),
            engine_x: f("sim/flightmodel2/engines/location_x_mtr"),
            engine_y: f("sim/flightmodel2/engines/location_y_mtr"),
            engine_z: f("sim/flightmodel2/engines/location_z_mtr"),
            sun_heading: f("sim/graphics/scenery/sun_heading_degrees"),
            sun_pitch: f("sim/graphics/scenery/sun_pitch_degrees"),
            fov: f("sim/graphics/view/field_of_view_deg"),
            paused: f("sim/time/paused"),
            gear_deploy: f("sim/flightmodel2/gear/deploy_ratio"),
            flaps: f("sim/flightmodel2/controls/flap1_deploy_ratio").or_else(|| f("sim/flightmodel/controls/flaprat")),
            reversers: f("sim/flightmodel2/engines/thrust_reverser_deploy_ratio"),
            speedbrake: f("sim/flightmodel2/controls/speedbrake_ratio"),
            landing_lights: f("sim/flightmodel2/lights/landing_lights_brightness_ratio"),
            landing_light_switch: f("sim/cockpit2/switches/landing_lights_on"),
            doors: f("sim/flightmodel2/misc/door_open_ratio"),
            canopy: f("sim/flightmodel2/misc/canopy_open_ratio"),
            engines_burning: f("sim/flightmodel2/engines/engine_is_burning_fuel"),
            wheel_x: f("sim/flightmodel/parts/tire_x_no_deflection"),
            wheel_y: f("sim/flightmodel/parts/tire_y_no_deflection"),
            wheel_z: f("sim/flightmodel/parts/tire_z_no_deflection"),
        }
    }

    fn subject(&self, size: f64) -> Subject {
        let d = |r: Option<DataRef>| r.map_or(0.0, DataRef::f64);
        let f = |r: Option<DataRef>| r.map_or(0.0, |r| r.f32() as f64);
        Subject {
            pos: Vec3::new(d(self.x), d(self.y), d(self.z)),
            vel: Vec3::new(f(self.vx), f(self.vy), f(self.vz)),
            att: Attitude::new(f(self.psi), f(self.theta), f(self.phi)),
            agl: f(self.agl),
            on_ground: self.on_ground.is_some_and(|r| r.i32() != 0),
            size,
            systems: self.systems(),
        }
    }

    /// Gear, flaps and reversers. Missing datarefs read as gear down, flaps
    /// up and reversers stowed, so they never trigger anything.
    fn systems(&self) -> Systems {
        let max = |r: Option<DataRef>, n: usize| {
            r.and_then(|r| r.f32s(n).into_iter().map(f64::from).filter(|v| v.is_finite()).reduce(f64::max))
        };
        let n_engines = self.num_engines.map_or(0, |n| n.i32().clamp(0, 16) as usize);
        let d = Systems::default();
        let lights = max(self.landing_lights, 16)
            .or_else(|| self.landing_light_switch.map(|r| if r.i32() != 0 { 1.0 } else { 0.0 }));
        let doors = [max(self.doors, 20), self.canopy.map(|r| r.f32() as f64)].into_iter().flatten().reduce(f64::max);
        let engines_running = self.engines_burning.map_or(0, |r| {
            r.i32s(n_engines).iter().enumerate().filter(|(_, &on)| on != 0).fold(0u16, |bits, (i, _)| bits | 1 << i)
        });
        Systems {
            // The first wheel to move: extending starts with it, retracting ends with it.
            gear: max(self.gear_deploy, 10).unwrap_or(d.gear),
            flaps: self.flaps.map_or(d.flaps, |r| r.f32() as f64),
            reverser: max(self.reversers, n_engines).unwrap_or(d.reverser),
            speedbrake: self.speedbrake.map_or(d.speedbrake, |r| r.f32() as f64),
            landing_lights: lights.unwrap_or(d.landing_lights),
            doors: doors.unwrap_or(d.doors),
            engines_running,
        }
    }

    /// Wheel positions, body frame, meters from the CG (+x right, +y up, +z aft).
    fn wheels(&self) -> Vec<Vec3> {
        let (Some(x), Some(y), Some(z)) = (self.wheel_x, self.wheel_y, self.wheel_z) else {
            return Vec::new();
        };
        let (xs, ys, zs) = (x.f32s(10), y.f32s(10), z.f32s(10));
        (0..xs.len().min(ys.len()).min(zs.len()))
            .map(|i| Vec3::new(xs[i] as f64, ys[i] as f64, zs[i] as f64))
            // Unused slots are at the origin.
            .filter(|w| w.x.is_finite() && w.y.is_finite() && w.z.is_finite() && w.len() > 0.1 && w.len() < 100.0)
            .collect()
    }

    /// Engine positions, body frame, meters from the CG (+x right, +y up,
    /// +z aft), with X-Plane's index for each.
    fn engines(&self) -> Vec<(usize, Vec3)> {
        let (Some(n), Some(x), Some(y), Some(z)) = (self.num_engines, self.engine_x, self.engine_y, self.engine_z) else {
            return Vec::new();
        };
        let n = n.i32().clamp(0, 16) as usize;
        let (xs, ys, zs) = (x.f32s(n), y.f32s(n), z.f32s(n));
        let engines: Vec<(usize, Vec3)> = (0..n.min(xs.len()).min(ys.len()).min(zs.len()))
            .map(|i| (i, Vec3::new(xs[i] as f64, ys[i] as f64, zs[i] as f64)))
            .filter(|(_, e)| e.x.is_finite() && e.y.is_finite() && e.z.is_finite() && e.len() < 100.0)
            .collect();
        // All at the origin means the aircraft doesn't say.
        if engines.iter().all(|(_, e)| e.len() < 0.1) {
            Vec::new()
        } else {
            engines
        }
    }
}

struct MenuUi {
    main: Menu,
    toggle: i32,
    settings: i32,
}

pub struct App {
    director: Director,
    refs: Refs,
    probe: Option<Probe>,
    window: EguiWindow,
    menu: Option<MenuUi>,
    /// Settings changed in the window and not yet written to disk.
    unsaved_since: Option<Instant>,
    /// "34 m wingspan · 2 engines", for the window footer.
    aircraft: String,
    prefs_path: PathBuf,
    state: State,
    /// View type before we engaged, to go back to it afterwards.
    saved_view: Option<i32>,
    size: f64,
    /// When the camera last moved: real time, so it keeps going while paused.
    last_time: Option<Instant>,
    /// When the last screenshot/movie command ran (real time, so it works while paused too).
    capture_at: Option<Instant>,
    /// Re-engaging after a capture: carry on with the same shot.
    resume: bool,
    /// The last view command seen while running, and when.
    view_cmd: Option<(usize, Instant)>,
    /// view_type when we last took the camera, for the log.
    engaged_view: Option<i32>,
    /// When we took the camera back after an unexplained loss.
    retakes: Vec<Instant>,
}

impl App {
    pub fn new(window: EguiWindow, handler: crate::ffi::XPLMMenuHandler_f) -> Self {
        view::install_style(window.context());
        let prefs_path = xp::system_path().join("Output").join("preferences").join("camship.prf");
        let settings = match fs::read_to_string(&prefs_path) {
            Ok(text) => Settings::parse(&text),
            Err(_) => Settings::default(),
        };
        let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(1, |d| d.as_nanos() as u64);
        let director = Director::new(settings, seed);
        let menu = build_menu(handler);
        if menu.is_none() {
            xp::log("could not create the menu");
        }
        let probe = Probe::new();
        if probe.is_none() {
            xp::log("could not create a terrain probe; ground clearance is off");
        }
        let mut app = Self {
            director,
            refs: Refs::find(),
            probe,
            window,
            menu,
            unsaved_since: None,
            aircraft: String::new(),
            prefs_path,
            state: State::Off,
            saved_view: None,
            size: DEFAULT_SIZE_M,
            last_time: None,
            capture_at: None,
            resume: false,
            view_cmd: None,
            engaged_view: None,
            retakes: Vec::new(),
        };
        app.refresh_size();
        app.sync_menu();
        app
    }

    pub fn handle(&mut self, ev: Event) -> Request {
        let req = match ev {
            Event::Toggle => match self.state {
                State::Off => self.start(),
                State::Starting | State::Running => self.stop(true),
            },
            Event::TakeControl => {
                if self.state == State::Starting {
                    if !std::mem::take(&mut self.resume) {
                        self.director.reset();
                        self.last_time = None;
                    }
                    if xp::camera_controlled() {
                        xp::log("another plugin is controlling the camera; taking it over");
                    }
                    self.state = State::Running;
                    self.view_cmd = None;
                    self.engaged_view = self.view_type();
                    Request::ControlCamera
                } else {
                    Request::None
                }
            }
            Event::ViewCommand(i) => {
                if self.state == State::Running {
                    self.view_cmd = Some((i, Instant::now()));
                }
                Request::None
            }
            Event::Capture => {
                self.capture_at = Some(Instant::now());
                Request::None
            }
            Event::LostControl => {
                let after_capture = self.capture_at.is_some_and(|t| t.elapsed().as_secs_f64() < CAPTURE_GRACE_S);
                let views = format!("view_type {} -> {}", fmt_view(self.engaged_view), fmt_view(self.view_type()));
                let view_cmd = self.view_cmd.take().filter(|(_, t)| t.elapsed() < VIEW_CMD_WINDOW);
                if self.state != State::Running {
                    Request::None
                } else if after_capture {
                    // X-Plane took the camera back around a screenshot or
                    // movie; resume the same shot as if nothing happened.
                    xp::log("camera lost during a screenshot/movie; taking it back");
                    self.capture_at = None;
                    self.retake()
                } else if let Some((i, _)) = view_cmd {
                    // The user chose another view; leave them there.
                    self.state = State::Off;
                    xp::log(&format!("camera released: {} changed the view ({views})", VIEW_COMMANDS[i]));
                    Request::None
                } else {
                    // No view command: another plugin took the camera.
                    self.retakes.retain(|t| t.elapsed() < RETAKE_WINDOW);
                    if self.retakes.len() < MAX_RETAKES {
                        self.retakes.push(Instant::now());
                        xp::log(&format!(
                            "camera lost without a view command ({views}); taking it back ({}/{MAX_RETAKES})",
                            self.retakes.len()
                        ));
                        self.retake()
                    } else {
                        self.state = State::Off;
                        xp::log(&format!(
                            "camera lost without a view command ({views}), {} times in {} s; another plugin keeps taking it, giving up",
                            MAX_RETAKES + 1,
                            RETAKE_WINDOW.as_secs()
                        ));
                        Request::None
                    }
                }
            }
            Event::NewFlight => {
                self.refresh_size();
                if self.state == State::Off {
                    Request::None
                } else {
                    self.stop(false)
                }
            }
            Event::NextShot => {
                if self.state == State::Off {
                    self.start()
                } else {
                    self.director.next_shot();
                    Request::None
                }
            }
            Event::ToggleHold => {
                let hold = !self.director.hold();
                self.director.set_hold(hold);
                Request::None
            }
            Event::SetHold(hold) => {
                self.director.set_hold(hold);
                Request::None
            }
            Event::Play(i) => {
                if self.state != State::Off {
                    self.director.play(i);
                }
                Request::None
            }
            Event::ToggleWindow => {
                let show = !self.window.is_visible();
                self.window.set_visible(show);
                if !show {
                    self.save_if_dirty(true);
                }
                Request::None
            }
        };
        self.sync_menu();
        req
    }

    fn start(&mut self) -> Request {
        self.saved_view = self.refs.view_type.map(DataRef::i32);
        self.refresh_size();
        // The exterior model is only drawn in an outside view.
        if !xp::command_once("sim/view/chase") && !xp::command_once("sim/view/circle") {
            xp::log("no outside view command found");
        }
        self.state = State::Starting;
        self.retakes.clear();
        Request::TakeControlNextFrame
    }

    /// Takes the camera back and carries on with the same shot.
    fn retake(&mut self) -> Request {
        self.resume = true;
        xp::command_once("sim/view/chase");
        self.state = State::Starting;
        Request::TakeControlNextFrame
    }

    fn view_type(&self) -> Option<i32> {
        self.refs.view_type.map(DataRef::i32)
    }

    fn stop(&mut self, restore_view: bool) -> Request {
        let was_running = self.state == State::Running;
        // Set first: releasing the camera may call back into us with LostControl.
        self.state = State::Off;
        if restore_view {
            // view_type values from X-Plane's DataRefs.txt.
            let cmd = match self.saved_view {
                Some(1026) => Some("sim/view/3d_cockpit_cmnd_look"),
                Some(1000) => Some("sim/view/forward_with_2d_panel"),
                Some(1023) => Some("sim/view/forward_with_hud"),
                _ => None,
            };
            if let Some(cmd) = cmd {
                xp::command_once(cmd);
            }
        }
        if was_running {
            Request::ReleaseCamera
        } else {
            Request::None
        }
    }

    fn save_if_dirty(&mut self, now: bool) {
        if self.unsaved_since.is_some_and(|t| now || t.elapsed() >= SAVE_DELAY) {
            self.unsaved_since = None;
            self.save();
        }
    }

    fn save(&self) {
        if let Err(e) = fs::write(&self.prefs_path, self.director.settings().to_text()) {
            xp::log(&format!("could not save {}: {e}", self.prefs_path.display()));
        }
    }

    fn refresh_size(&mut self) {
        let configured = self.director.settings().subject_size_m;
        self.size = if configured > 0.0 {
            configured
        } else {
            let x = self.refs.size_x.map_or(0.0, |r| r.f32() as f64);
            let z = self.refs.size_z.map_or(0.0, |r| r.f32() as f64);
            let s = x.max(z);
            if s.is_finite() && s > 1.0 {
                s.clamp(6.0, 90.0)
            } else {
                DEFAULT_SIZE_M
            }
        };
        xp::log(&format!("subject size {:.1} m (set subject_size_m in camship.prf to override)", self.size));
        let engines = self.refs.engines();
        match engines.len() {
            0 => xp::log("no engine positions reported; engine close-ups are off for this aircraft"),
            n => xp::log(&format!(
                "{n} engine(s) at {}",
                engines.iter().map(|(_, e)| format!("({:.1}, {:.1}, {:.1})", e.x, e.y, e.z)).collect::<Vec<_>>().join(", ")
            )),
        }
        self.aircraft = format!(
            "This aircraft: {:.0} m across · {}",
            self.size,
            match engines.len() {
                0 => "no engine positions reported".to_string(),
                1 => "1 engine".to_string(),
                n => format!("{n} engines"),
            }
        );
        let (slots, positions): (Vec<usize>, Vec<Vec3>) = engines.into_iter().unzip();
        self.director.set_engines(positions);
        self.director.set_engine_slots(slots);
        let wheels = self.refs.wheels();
        match wheels.len() {
            0 => xp::log("no wheel positions reported; gear shots use a typical layout"),
            n => xp::log(&format!(
                "{n} wheel(s) at {}",
                wheels.iter().map(|w| format!("({:.1}, {:.1}, {:.1})", w.x, w.y, w.z)).collect::<Vec<_>>().join(", ")
            )),
        }
        self.director.set_gear(wheels);
    }

    pub fn window(&mut self) -> &mut EguiWindow {
        &mut self.window
    }

    /// Draw callback of our window. Returns what the user clicked, to be
    /// posted as events once we're out of the app.
    pub fn draw(&mut self) -> Vec<Event> {
        let shots = self.director.shots();
        let supported: Vec<bool> = (0..shots.len()).map(|i| self.director.supported(i, self.size)).collect();
        let running = self.state != State::Off;
        let current = self.director.current_shot();
        let hold = self.director.hold();
        let before = self.director.settings().clone();
        let mut settings = before.clone();

        let mut actions = Vec::new();
        let mut vm = ViewModel {
            settings: &mut settings,
            shots,
            supported: &supported,
            running,
            current,
            hold,
            aircraft: &self.aircraft,
        };
        self.window.draw(|ui, popped_out| view::show(ui, &mut vm, popped_out, &mut actions));

        if settings != before {
            *self.director.settings_mut() = settings;
            self.unsaved_since = Some(Instant::now());
        }
        self.save_if_dirty(false);

        actions
            .into_iter()
            .map(|a| match a {
                Action::Toggle => Event::Toggle,
                Action::Next => Event::NextShot,
                Action::SetHold(h) => Event::SetHold(h),
                Action::Play(i) => Event::Play(i),
            })
            .collect()
    }

    /// One frame of the camera, or `None` to give the camera back.
    pub fn camera_frame(&mut self) -> Option<CameraPose> {
        if self.state != State::Running {
            return None;
        }
        let now = Instant::now();
        let mut dt = self.last_time.map_or(0.0, |t| now.duration_since(t).as_secs_f64());
        self.last_time = Some(now);
        let paused = self.refs.paused.is_some_and(|r| r.i32() != 0);
        if paused && !self.director.settings().film_when_paused {
            dt = 0.0;
        }
        let subject = self.refs.subject(self.size);
        if let (Some(h), Some(p)) = (self.refs.sun_heading, self.refs.sun_pitch) {
            self.director.set_sun(h.f32() as f64, p.f32() as f64);
        }
        if let Some(fov) = self.refs.fov {
            self.director.set_fov(fov.f32() as f64);
        }
        let probe = self.probe.as_ref();
        let mut ground = |p: Vec3| probe.and_then(|pr| pr.ground_y(p.x, p.y, p.z));
        let pose = self.director.update(&subject, dt, &mut ground);
        for note in self.director.take_notes() {
            xp::log(&note);
        }
        Some(pose)
    }

    pub fn shutdown(&mut self) -> Request {
        let req = if self.state == State::Off { Request::None } else { self.stop(false) };
        self.unsaved_since = None;
        self.save();
        req
    }

    fn sync_menu(&self) {
        let Some(m) = &self.menu else { return };
        m.main.set_checked(m.toggle, self.state != State::Off);
        m.main.set_checked(m.settings, self.window.is_visible());
    }
}

fn fmt_view(v: Option<i32>) -> String {
    v.map_or_else(|| "?".to_string(), |v| v.to_string())
}

fn build_menu(handler: crate::ffi::XPLMMenuHandler_f) -> Option<MenuUi> {
    let main = Menu::in_plugins_menu("Camship", handler)?;
    let toggle = main.item("Cinematic camera", MENU_TOGGLE);
    main.item("Next shot", MENU_NEXT);
    main.separator();
    let settings = main.item("Settings...", MENU_SETTINGS);
    Some(MenuUi { main, toggle, settings })
}
