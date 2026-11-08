//! Hand-written bindings for the small subset of the X-Plane SDK (XPLM 2.1+)
//! Camship uses.
//!
//! Signatures and constants mirror the official C headers. On Windows we link
//! with `raw-dylib`, so no SDK import libraries are needed to build; on macOS
//! and Linux the symbols are resolved by X-Plane when the plugin is loaded
//! (see `build.rs`).

#![allow(non_camel_case_types, non_snake_case, dead_code)]

use std::ffi::{c_char, c_float, c_int, c_void};

pub type XPLMDataRef = *mut c_void;
pub type XPLMCommandRef = *mut c_void;
pub type XPLMMenuID = *mut c_void;
pub type XPLMPluginID = c_int;
pub type XPLMProbeRef = *mut c_void;
pub type XPLMWindowID = *mut c_void;

pub type XPLMFlightLoop_f =
    unsafe extern "C" fn(since_last_call: c_float, since_last_loop: c_float, counter: c_int, refcon: *mut c_void) -> c_float;
pub type XPLMMenuHandler_f = unsafe extern "C" fn(menu_ref: *mut c_void, item_ref: *mut c_void);
pub type XPLMCommandCallback_f = unsafe extern "C" fn(cmd: XPLMCommandRef, phase: c_int, refcon: *mut c_void) -> c_int;
pub type XPLMCameraControl_f =
    unsafe extern "C" fn(out: *mut XPLMCameraPosition_t, is_losing_control: c_int, refcon: *mut c_void) -> c_int;
pub type XPLMDrawWindow_f = unsafe extern "C" fn(win: XPLMWindowID, refcon: *mut c_void);
pub type XPLMDrawCallback_f = unsafe extern "C" fn(phase: c_int, is_before: c_int, refcon: *mut c_void) -> c_int;
pub type XPLMHandleKey_f =
    unsafe extern "C" fn(win: XPLMWindowID, key: c_char, flags: c_int, vkey: c_char, refcon: *mut c_void, losing_focus: c_int);
pub type XPLMHandleMouseClick_f =
    unsafe extern "C" fn(win: XPLMWindowID, x: c_int, y: c_int, status: c_int, refcon: *mut c_void) -> c_int;
pub type XPLMHandleCursor_f = unsafe extern "C" fn(win: XPLMWindowID, x: c_int, y: c_int, refcon: *mut c_void) -> c_int;
pub type XPLMHandleMouseWheel_f =
    unsafe extern "C" fn(win: XPLMWindowID, x: c_int, y: c_int, wheel: c_int, clicks: c_int, refcon: *mut c_void) -> c_int;

#[repr(C)]
pub struct XPLMCreateWindow_t {
    pub structSize: c_int,
    pub left: c_int,
    pub top: c_int,
    pub right: c_int,
    pub bottom: c_int,
    pub visible: c_int,
    pub drawWindowFunc: XPLMDrawWindow_f,
    pub handleMouseClickFunc: XPLMHandleMouseClick_f,
    pub handleKeyFunc: XPLMHandleKey_f,
    pub handleCursorFunc: XPLMHandleCursor_f,
    pub handleMouseWheelFunc: XPLMHandleMouseWheel_f,
    pub refcon: *mut c_void,
    pub decorateAsFloatingWindow: c_int,
    pub layer: c_int,
    pub handleRightClickFunc: XPLMHandleMouseClick_f,
}


// XPLMCamera
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct XPLMCameraPosition_t {
    pub x: c_float,
    pub y: c_float,
    pub z: c_float,
    pub pitch: c_float,
    pub heading: c_float,
    pub roll: c_float,
    pub zoom: c_float,
}
pub const xplm_ControlCameraUntilViewChanges: c_int = 1;
pub const xplm_ControlCameraForever: c_int = 2;
pub const xplm_Phase_Window: c_int = 50;

// XPLMScenery
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct XPLMProbeInfo_t {
    pub structSize: c_int,
    pub locationX: c_float,
    pub locationY: c_float,
    pub locationZ: c_float,
    pub normalX: c_float,
    pub normalY: c_float,
    pub normalZ: c_float,
    pub velocityX: c_float,
    pub velocityY: c_float,
    pub velocityZ: c_float,
    pub is_wet: c_int,
}
pub const xplm_ProbeY: c_int = 0;
pub const xplm_ProbeHitTerrain: c_int = 0;

// XPLMDisplay
pub const xplm_WindowDecorationRoundRectangle: c_int = 1;
pub const xplm_WindowLayerFloatingWindows: c_int = 1;
pub const xplm_WindowPositionFree: c_int = 0;
pub const xplm_MouseDown: c_int = 1;
pub const xplm_MouseDrag: c_int = 2;
pub const xplm_MouseUp: c_int = 3;
pub const xplm_CursorDefault: c_int = 0;

// XPLMDefs: key flags and virtual keys
pub const xplm_ShiftFlag: c_int = 1;
pub const xplm_OptionAltFlag: c_int = 2;
pub const xplm_ControlFlag: c_int = 4;
pub const xplm_DownFlag: c_int = 8;
pub const xplm_UpFlag: c_int = 16;
pub const XPLM_VK_BACK: u8 = 0x08;
pub const XPLM_VK_TAB: u8 = 0x09;
pub const XPLM_VK_RETURN: u8 = 0x0D;
pub const XPLM_VK_ESCAPE: u8 = 0x1B;
pub const XPLM_VK_END: u8 = 0x23;
pub const XPLM_VK_HOME: u8 = 0x24;
pub const XPLM_VK_LEFT: u8 = 0x25;
pub const XPLM_VK_UP: u8 = 0x26;
pub const XPLM_VK_RIGHT: u8 = 0x27;
pub const XPLM_VK_DOWN: u8 = 0x28;
pub const XPLM_VK_DELETE: u8 = 0x2E;
pub const XPLM_VK_ENTER: u8 = 0xBB;
pub const XPLM_VK_NUMPAD_ENT: u8 = 0xBC;

// XPLMMenus
pub const xplm_Menu_Unchecked: c_int = 1;
pub const xplm_Menu_Checked: c_int = 2;

// XPLMUtilities
pub const xplm_CommandBegin: c_int = 0;

// XPLMPlugin
pub const XPLM_MSG_PLANE_LOADED: c_int = 102;
pub const XPLM_MSG_AIRPORT_LOADED: c_int = 103;

#[cfg_attr(windows, link(name = "XPLM_64", kind = "raw-dylib"))]
extern "C" {
    // XPLMUtilities
    pub fn XPLMDebugString(s: *const c_char);
    pub fn XPLMGetSystemPath(out: *mut c_char);
    pub fn XPLMFindCommand(name: *const c_char) -> XPLMCommandRef;
    pub fn XPLMCommandOnce(cmd: XPLMCommandRef);
    pub fn XPLMCreateCommand(name: *const c_char, desc: *const c_char) -> XPLMCommandRef;
    pub fn XPLMRegisterCommandHandler(cmd: XPLMCommandRef, h: XPLMCommandCallback_f, before: c_int, refcon: *mut c_void);
    pub fn XPLMUnregisterCommandHandler(cmd: XPLMCommandRef, h: XPLMCommandCallback_f, before: c_int, refcon: *mut c_void);

    // XPLMPlugin
    pub fn XPLMEnableFeature(feature: *const c_char, enable: c_int);

    // XPLMCamera
    pub fn XPLMControlCamera(how_long: c_int, func: XPLMCameraControl_f, refcon: *mut c_void);
    pub fn XPLMDontControlCamera();
    pub fn XPLMIsCameraBeingControlled(duration: *mut c_int) -> c_int;

    // XPLMScenery
    pub fn XPLMCreateProbe(kind: c_int) -> XPLMProbeRef;
    pub fn XPLMDestroyProbe(probe: XPLMProbeRef);
    pub fn XPLMProbeTerrainXYZ(probe: XPLMProbeRef, x: c_float, y: c_float, z: c_float, info: *mut XPLMProbeInfo_t) -> c_int;

    // XPLMDisplay
    pub fn XPLMGetScreenBoundsGlobal(l: *mut c_int, t: *mut c_int, r: *mut c_int, b: *mut c_int);
    pub fn XPLMCreateWindowEx(params: *mut XPLMCreateWindow_t) -> XPLMWindowID;
    pub fn XPLMDestroyWindow(win: XPLMWindowID);
    pub fn XPLMGetWindowGeometry(win: XPLMWindowID, l: *mut c_int, t: *mut c_int, r: *mut c_int, b: *mut c_int);
    pub fn XPLMSetWindowGeometry(win: XPLMWindowID, l: c_int, t: c_int, r: c_int, b: c_int);
    pub fn XPLMGetWindowIsVisible(win: XPLMWindowID) -> c_int;
    pub fn XPLMSetWindowIsVisible(win: XPLMWindowID, visible: c_int);
    pub fn XPLMSetWindowTitle(win: XPLMWindowID, title: *const c_char);
    pub fn XPLMSetWindowResizingLimits(win: XPLMWindowID, min_w: c_int, min_h: c_int, max_w: c_int, max_h: c_int);
    pub fn XPLMSetWindowPositioningMode(win: XPLMWindowID, mode: c_int, monitor: c_int);
    pub fn XPLMBringWindowToFront(win: XPLMWindowID);
    pub fn XPLMWindowIsPoppedOut(win: XPLMWindowID) -> c_int;
    pub fn XPLMTakeKeyboardFocus(win: XPLMWindowID);
    pub fn XPLMHasKeyboardFocus(win: XPLMWindowID) -> c_int;
    pub fn XPLMGetMouseLocationGlobal(x: *mut c_int, y: *mut c_int);

    // XPLMGraphics
    pub fn XPLMSetGraphicsState(
        fog: c_int,
        tex_units: c_int,
        lighting: c_int,
        alpha_testing: c_int,
        alpha_blending: c_int,
        depth_testing: c_int,
        depth_writing: c_int,
    );

    // XPLMDataAccess
    pub fn XPLMFindDataRef(name: *const c_char) -> XPLMDataRef;
    pub fn XPLMGetDatai(r: XPLMDataRef) -> c_int;
    pub fn XPLMGetDataf(r: XPLMDataRef) -> c_float;
    pub fn XPLMGetDatad(r: XPLMDataRef) -> f64;
    pub fn XPLMGetDatavf(r: XPLMDataRef, out: *mut c_float, offset: c_int, max: c_int) -> c_int;
    pub fn XPLMGetDatavi(r: XPLMDataRef, out: *mut c_int, offset: c_int, max: c_int) -> c_int;

    // XPLMProcessing
    pub fn XPLMRegisterFlightLoopCallback(f: XPLMFlightLoop_f, interval: c_float, refcon: *mut c_void);
    pub fn XPLMUnregisterFlightLoopCallback(f: XPLMFlightLoop_f, refcon: *mut c_void);
    pub fn XPLMSetFlightLoopCallbackInterval(f: XPLMFlightLoop_f, interval: c_float, relative_to_now: c_int, refcon: *mut c_void);

    // XPLMDisplay: drawing
    pub fn XPLMRegisterDrawCallback(f: XPLMDrawCallback_f, phase: c_int, before: c_int, refcon: *mut c_void) -> c_int;
    pub fn XPLMUnregisterDrawCallback(f: XPLMDrawCallback_f, phase: c_int, before: c_int, refcon: *mut c_void) -> c_int;

    // XPLMMenus
    pub fn XPLMFindPluginsMenu() -> XPLMMenuID;
    pub fn XPLMCreateMenu(
        name: *const c_char,
        parent: XPLMMenuID,
        parent_item: c_int,
        handler: Option<XPLMMenuHandler_f>,
        menu_ref: *mut c_void,
    ) -> XPLMMenuID;
    pub fn XPLMDestroyMenu(menu: XPLMMenuID);
    pub fn XPLMAppendMenuItem(menu: XPLMMenuID, name: *const c_char, item_ref: *mut c_void, deprecated: c_int) -> c_int;
    pub fn XPLMAppendMenuSeparator(menu: XPLMMenuID);
    pub fn XPLMRemoveMenuItem(menu: XPLMMenuID, index: c_int);
    pub fn XPLMCheckMenuItem(menu: XPLMMenuID, index: c_int, check: c_int);
}

// OpenGL entry points come from the driver X-Plane already loaded.
#[cfg(windows)]
#[link(name = "opengl32", kind = "raw-dylib")]
extern "system" {
    pub fn wglGetProcAddress(name: *const c_char) -> *const c_void;
}

#[cfg(windows)]
#[link(name = "kernel32", kind = "raw-dylib")]
extern "system" {
    pub fn GetModuleHandleA(name: *const c_char) -> *mut c_void;
    pub fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *const c_void;
}

#[cfg(target_os = "linux")]
#[link(name = "dl")]
extern "C" {
    pub fn dlopen(file: *const c_char, mode: c_int) -> *mut c_void;
    pub fn dlsym(handle: *mut c_void, name: *const c_char) -> *const c_void;
}

#[cfg(target_os = "macos")]
extern "C" {
    pub fn dlsym(handle: *mut c_void, name: *const c_char) -> *const c_void;
}
