//! Plugin state: engaging/releasing X-Plane's camera, feeding the director,
//! the settings window, the menu and preferences.

use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use camship_core::{Attitude, CameraPose, Director, Settings, Subject, Vec3};

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
    /// X-Plane took the camera back (the user picked another view).
    LostControl,
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
    time: Option<DataRef>,
    size_x: Option<DataRef>,
    size_z: Option<DataRef>,
    view_type: Option<DataRef>,
    num_engines: Option<DataRef>,
    engine_x: Option<DataRef>,
    engine_y: Option<DataRef>,
    engine_z: Option<DataRef>,
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
            time: f("sim/time/total_running_time_sec"),
            size_x: f("sim/aircraft/view/acf_size_x"),
            size_z: f("sim/aircraft/view/acf_size_z"),
            view_type: f("sim/graphics/view/view_type"),
            num_engines: f("sim/aircraft/engine/acf_num_engines"),
            engine_x: f("sim/flightmodel2/engines/location_x_mtr"),
            engine_y: f("sim/flightmodel2/engines/location_y_mtr"),
            engine_z: f("sim/flightmodel2/engines/location_z_mtr"),
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
        }
    }

    /// Engine positions, body frame, meters from the CG (+x right, +y up, +z aft).
    fn engines(&self) -> Vec<Vec3> {
        let (Some(n), Some(x), Some(y), Some(z)) = (self.num_engines, self.engine_x, self.engine_y, self.engine_z) else {
            return Vec::new();
        };
        let n = n.i32().clamp(0, 16) as usize;
        let (xs, ys, zs) = (x.f32s(n), y.f32s(n), z.f32s(n));
        let engines: Vec<Vec3> = (0..n.min(xs.len()).min(ys.len()).min(zs.len()))
            .map(|i| Vec3::new(xs[i] as f64, ys[i] as f64, zs[i] as f64))
            .filter(|e| e.x.is_finite() && e.y.is_finite() && e.z.is_finite() && e.len() < 100.0)
            .collect();
        // All at the origin means the aircraft doesn't say.
        if engines.iter().all(|e| e.len() < 0.1) {
            Vec::new()
        } else {
            engines
        }
    }

    fn time(&self) -> f64 {
        self.time.map_or(0.0, |r| r.f32() as f64)
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
    last_time: Option<f64>,
    /// When the last screenshot/movie command ran (sim running time).
    capture_at: Option<f64>,
    /// Re-engaging after a capture: carry on with the same shot.
    resume: bool,
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
                    self.state = State::Running;
                    Request::ControlCamera
                } else {
                    Request::None
                }
            }
            Event::Capture => {
                self.capture_at = Some(self.refs.time());
                Request::None
            }
            Event::LostControl => {
                let after_capture = self.capture_at.is_some_and(|t| (self.refs.time() - t).abs() < CAPTURE_GRACE_S);
                if self.state != State::Running {
                    Request::None
                } else if after_capture {
                    // X-Plane took the camera back around a screenshot or
                    // movie; resume the same shot as if nothing happened.
                    xp::log("camera lost during a screenshot/movie; taking it back");
                    self.capture_at = None;
                    self.resume = true;
                    xp::command_once("sim/view/chase");
                    self.state = State::Starting;
                    Request::TakeControlNextFrame
                } else {
                    // The user chose another view; leave them there.
                    self.state = State::Off;
                    xp::log("camera released (view changed)");
                    Request::None
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
        Request::TakeControlNextFrame
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
                engines.iter().map(|e| format!("({:.1}, {:.1}, {:.1})", e.x, e.y, e.z)).collect::<Vec<_>>().join(", ")
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
        self.director.set_engines(engines);
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
        let now = self.refs.time();
        let dt = self.last_time.map_or(0.0, |t| (now - t).max(0.0));
        self.last_time = Some(now);
        let subject = self.refs.subject(self.size);
        let probe = self.probe.as_ref();
        let mut ground = |p: Vec3| probe.and_then(|pr| pr.ground_y(p.x, p.y, p.z));
        Some(self.director.update(&subject, dt, &mut ground))
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

fn build_menu(handler: crate::ffi::XPLMMenuHandler_f) -> Option<MenuUi> {
    let main = Menu::in_plugins_menu("Camship", handler)?;
    let toggle = main.item("Cinematic camera", MENU_TOGGLE);
    main.item("Next shot", MENU_NEXT);
    main.separator();
    let settings = main.item("Settings...", MENU_SETTINGS);
    Some(MenuUi { main, toggle, settings })
}
