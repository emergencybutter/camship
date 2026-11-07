//! Camship: a cinematic camera for X-Plane that cuts between moving
//! air-to-air and ground shots, like a camera ship filming a commercial.
//!
//! This file holds the C entry points X-Plane calls. Every callback is wrapped
//! in [`guard`] so a Rust panic is logged instead of crashing the simulator,
//! and every callback goes into [`App`] through [`post`] / [`with_app`],
//! which serialise access even when X-Plane calls us back re-entrantly (e.g.
//! releasing the camera synchronously calls the camera callback).

#![allow(non_upper_case_globals)]
#![allow(clippy::missing_safety_doc)]

mod app;
mod ffi;
mod gl;
mod gui;
mod view;
mod xp;

use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::ffi::{c_char, c_float, c_int, c_void};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;

use app::*;
use ffi::*;

thread_local! {
    static APP: RefCell<Option<App>> = const { RefCell::new(None) };
    static QUEUE: RefCell<VecDeque<Event>> = const { RefCell::new(VecDeque::new()) };
    static BUSY: Cell<bool> = const { Cell::new(false) };
    static COMMANDS: RefCell<Vec<xp::OwnedCommand>> = const { RefCell::new(Vec::new()) };
    /// Frames drawn so far, counted by the draw callback.
    static FRAME: Cell<u64> = const { Cell::new(0) };
    /// Take the camera once this frame is drawn (None: nothing pending).
    static TAKE_AT: Cell<Option<u64>> = const { Cell::new(None) };
}

const CMD_TOGGLE: usize = 1;
const CMD_NEXT: usize = 2;
const CMD_HOLD: usize = 3;
const CMD_WINDOW: usize = 4;
/// X-Plane's screenshot / movie commands, watched but never swallowed.
const CMD_CAPTURE: usize = 10;
/// `CMD_VIEW + i` watches `VIEW_COMMANDS[i]`, never swallowed.
const CMD_VIEW: usize = 100;

fn guard<R>(what: &str, fallback: R, f: impl FnOnce() -> R) -> R {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or_else(|e| {
        let msg = e
            .downcast_ref::<&str>()
            .map(|s| s.to_string())
            .or_else(|| e.downcast_ref::<String>().cloned())
            .unwrap_or_default();
        xp::log(&format!("internal error in {what}: {msg}"));
        fallback
    })
}

/// Clears the busy flag even if the work panicked.
struct BusyGuard;
impl Drop for BusyGuard {
    fn drop(&mut self) {
        BUSY.with(|b| b.set(false));
    }
}

/// Runs `f` on the app, then drains queued events. Returns `None` if we are
/// already inside the app (re-entrant call); queued events are then left for
/// the outer call.
fn with_app<R>(f: impl FnOnce(&mut App) -> R) -> Option<R> {
    if BUSY.with(|b| b.replace(true)) {
        return None;
    }
    let _busy = BusyGuard;
    let out = APP.with(|a| a.borrow_mut().as_mut().map(f));
    while let Some(ev) = QUEUE.with(|q| q.borrow_mut().pop_front()) {
        let req = APP.with(|a| a.borrow_mut().as_mut().map(|app| app.handle(ev)));
        if let Some(req) = req {
            perform(req);
        }
    }
    out
}

fn post(ev: Event) {
    QUEUE.with(|q| q.borrow_mut().push_back(ev));
    with_app(|_| ());
}

/// Carries out what the app asked for. Runs outside the app borrow, so any
/// synchronous callback from X-Plane only gets queued.
fn perform(req: Request) {
    unsafe {
        match req {
            Request::None => {}
            Request::TakeControlNextFrame => {
                // The flight loop is the usual route, but it doesn't run while
                // the sim is paused; the draw callback covers that case.
                // Whichever fires first takes the camera.
                TAKE_AT.with(|t| t.set(Some(FRAME.with(Cell::get) + 2)));
                XPLMSetFlightLoopCallbackInterval(take_control_loop, -1.0, 1, ptr::null_mut());
            }
            Request::ControlCamera => XPLMControlCamera(xplm_ControlCameraUntilViewChanges, camera, ptr::null_mut()),
            Request::ReleaseCamera => XPLMDontControlCamera(),
        }
    }
}

fn copy_out(dst: *mut c_char, s: &str) {
    // X-Plane gives us 256-byte buffers.
    let bytes = s.as_bytes();
    let n = bytes.len().min(255);
    unsafe {
        ptr::copy_nonoverlapping(bytes.as_ptr().cast::<c_char>(), dst, n);
        *dst.add(n) = 0;
    }
}

// ---------------------------------------------------------------------------
// Callbacks

unsafe extern "C" fn camera(out: *mut XPLMCameraPosition_t, losing: c_int, _: *mut c_void) -> c_int {
    let kept = guard("camera", None, || {
        if losing != 0 || out.is_null() {
            post(Event::LostControl);
            return Some(0);
        }
        match with_app(App::camera_frame) {
            Some(Some(pose)) => {
                unsafe {
                    *out = XPLMCameraPosition_t {
                        x: pose.pos.x as f32,
                        y: pose.pos.y as f32,
                        z: pose.pos.z as f32,
                        pitch: pose.pitch as f32,
                        heading: pose.heading as f32,
                        roll: pose.roll as f32,
                        zoom: pose.zoom as f32,
                    };
                }
                Some(1)
            }
            // We're off: give the camera back.
            Some(None) => Some(0),
            // Re-entrant: keep control, leave this frame's position as X-Plane passed it.
            None => Some(1),
        }
    });
    kept.unwrap_or_else(|| {
        // A panic: give the camera back so the user is never stuck.
        post(Event::LostControl);
        0
    })
}

/// Takes the camera if a request is pending (from either route below).
fn take_control_now() {
    if TAKE_AT.with(|t| t.take()).is_some() {
        post(Event::TakeControl);
    }
}

unsafe extern "C" fn take_control_loop(_: c_float, _: c_float, _: c_int, _: *mut c_void) -> c_float {
    guard("flight loop", 0.0, || {
        take_control_now();
        0.0
    })
}

/// Runs every frame, paused or not. Waits a full frame after the request,
/// like the flight loop, so the chase view is in place first.
unsafe extern "C" fn frame_tick(_: c_int, _: c_int, _: *mut c_void) -> c_int {
    guard("frame", 1, || {
        let frame = FRAME.with(|f| {
            f.set(f.get() + 1);
            f.get()
        });
        if TAKE_AT.with(Cell::get).is_some_and(|at| frame >= at) {
            take_control_now();
        }
        1
    })
}

unsafe extern "C" fn menu_handler(_: *mut c_void, item: *mut c_void) {
    guard("menu", (), || match item as usize {
        MENU_TOGGLE => post(Event::Toggle),
        MENU_NEXT => post(Event::NextShot),
        MENU_SETTINGS => post(Event::ToggleWindow),
        _ => {}
    })
}

unsafe extern "C" fn draw_window(_: XPLMWindowID, _: *mut c_void) {
    guard("draw", (), || {
        // Clicks become events, posted once we're out of the app.
        for ev in with_app(App::draw).unwrap_or_default() {
            post(ev);
        }
    })
}

unsafe extern "C" fn mouse_click(_: XPLMWindowID, x: c_int, y: c_int, status: c_int, _: *mut c_void) -> c_int {
    guard("mouse", 1, || {
        let consumed = with_app(|a| a.window().on_mouse(x, y, status, egui::PointerButton::Primary));
        consumed.unwrap_or(true) as c_int
    })
}

unsafe extern "C" fn right_click(_: XPLMWindowID, x: c_int, y: c_int, status: c_int, _: *mut c_void) -> c_int {
    guard("mouse", 1, || {
        let consumed = with_app(|a| a.window().on_mouse(x, y, status, egui::PointerButton::Secondary));
        consumed.unwrap_or(true) as c_int
    })
}

unsafe extern "C" fn key_press(_: XPLMWindowID, key: c_char, flags: c_int, vkey: c_char, _: *mut c_void, losing: c_int) {
    guard("key", (), || {
        with_app(|a| a.window().on_key(key, flags, vkey, losing != 0));
    })
}

unsafe extern "C" fn cursor(_: XPLMWindowID, _: c_int, _: c_int, _: *mut c_void) -> c_int {
    xplm_CursorDefault
}

unsafe extern "C" fn mouse_wheel(_: XPLMWindowID, x: c_int, y: c_int, wheel: c_int, clicks: c_int, _: *mut c_void) -> c_int {
    guard("wheel", 1, || {
        with_app(|a| a.window().on_wheel(x, y, wheel, clicks));
        1
    })
}

unsafe extern "C" fn command_handler(_: XPLMCommandRef, phase: c_int, refcon: *mut c_void) -> c_int {
    guard("command", 1, || {
        if phase == xplm_CommandBegin {
            match refcon as usize {
                CMD_TOGGLE => post(Event::Toggle),
                CMD_NEXT => post(Event::NextShot),
                CMD_HOLD => post(Event::ToggleHold),
                CMD_WINDOW => post(Event::ToggleWindow),
                CMD_CAPTURE => post(Event::Capture),
                r if r >= CMD_VIEW => post(Event::ViewCommand(r - CMD_VIEW)),
                _ => {}
            }
        }
        // Let X-Plane run its own commands; ours end here.
        (refcon as usize >= CMD_CAPTURE) as c_int
    })
}

// ---------------------------------------------------------------------------
// Plugin entry points

#[no_mangle]
pub unsafe extern "C" fn XPluginStart(name: *mut c_char, sig: *mut c_char, desc: *mut c_char) -> c_int {
    copy_out(name, "Camship");
    copy_out(sig, "com.github.camship");
    copy_out(desc, "Cinematic camera that cuts between moving air-to-air and ground shots.");
    guard("start", 0, || {
        xp::enable_feature("XPLM_USE_NATIVE_PATHS");
        let window = gui::EguiWindow::new(
            concat!("Camship v", env!("CARGO_PKG_VERSION")),
            440,
            640,
            gui::Callbacks {
                draw: draw_window,
                mouse: mouse_click,
                right_mouse: right_click,
                key: key_press,
                cursor,
                wheel: mouse_wheel,
            },
        );
        let app = App::new(window, menu_handler);
        APP.with(|a| *a.borrow_mut() = Some(app));

        COMMANDS.with(|c| {
            let mut c = c.borrow_mut();
            for (name, desc, refcon) in [
                ("camship/toggle", "Camship: start/stop the cinematic camera", CMD_TOGGLE),
                ("camship/next_shot", "Camship: cut to the next shot", CMD_NEXT),
                ("camship/hold", "Camship: hold/release the current shot", CMD_HOLD),
                ("camship/toggle_window", "Camship: show/hide the settings window", CMD_WINDOW),
            ] {
                c.push(xp::OwnedCommand::new(name, desc, command_handler, refcon));
            }
            // Shift+Space and the movie recorder must keep working on our camera.
            for name in ["sim/operation/screenshot", "sim/operation/video_record_toggle"] {
                match xp::OwnedCommand::watch(name, command_handler, CMD_CAPTURE) {
                    Some(w) => c.push(w),
                    None => xp::log(&format!("{name} not found")),
                }
            }
            // To tell the user picking a view from another plugin taking the camera.
            for (i, name) in VIEW_COMMANDS.iter().enumerate() {
                if let Some(w) = xp::OwnedCommand::watch(name, command_handler, CMD_VIEW + i) {
                    c.push(w);
                }
            }
        });
        xp::log(concat!("started v", env!("CARGO_PKG_VERSION")));
        1
    })
}

#[no_mangle]
pub unsafe extern "C" fn XPluginStop() {
    guard("stop", (), || {
        COMMANDS.with(|c| c.borrow_mut().clear());
        BUSY.with(|b| b.set(true));
        let _busy = BusyGuard;
        let app = APP.with(|a| a.borrow_mut().take());
        drop(app);
        QUEUE.with(|q| q.borrow_mut().clear());
    })
}

#[no_mangle]
pub unsafe extern "C" fn XPluginEnable() -> c_int {
    guard("enable", 0, || {
        // Registered idle (interval 0); scheduled on demand to take the camera.
        XPLMRegisterFlightLoopCallback(take_control_loop, 0.0, ptr::null_mut());
        XPLMRegisterDrawCallback(frame_tick, xplm_Phase_Window, 0, ptr::null_mut());
        1
    })
}

#[no_mangle]
pub unsafe extern "C" fn XPluginDisable() {
    guard("disable", (), || {
        if let Some(req) = with_app(App::shutdown) {
            perform(req);
        }
        XPLMUnregisterFlightLoopCallback(take_control_loop, ptr::null_mut());
        XPLMUnregisterDrawCallback(frame_tick, xplm_Phase_Window, 0, ptr::null_mut());
        TAKE_AT.with(|t| t.set(None));
    })
}

#[no_mangle]
pub unsafe extern "C" fn XPluginReceiveMessage(_from: XPLMPluginID, msg: c_int, param: *mut c_void) {
    guard("message", (), || {
        let user_plane = param.is_null();
        if msg == XPLM_MSG_AIRPORT_LOADED || (msg == XPLM_MSG_PLANE_LOADED && user_plane) {
            post(Event::NewFlight);
        }
    })
}
