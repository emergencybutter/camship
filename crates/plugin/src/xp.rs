//! Thin safe wrappers over [`crate::ffi`]. Everything here must be called from
//! X-Plane's main thread, which is the only thread plugins are called on.

use std::ffi::{c_char, c_int, CString};
use std::path::PathBuf;
use std::ptr;

use crate::ffi::*;

pub fn cstring(s: &str) -> CString {
    CString::new(s.replace('\0', "")).unwrap_or_default()
}

pub fn log(msg: &str) {
    let line = cstring(&format!("Camship: {msg}\n"));
    unsafe { XPLMDebugString(line.as_ptr()) }
}

pub fn enable_feature(name: &str) {
    let s = cstring(name);
    unsafe { XPLMEnableFeature(s.as_ptr(), 1) }
}

/// The X-Plane root folder (requires `XPLM_USE_NATIVE_PATHS` on macOS).
pub fn system_path() -> PathBuf {
    let mut buf = [0 as c_char; 1024];
    unsafe { XPLMGetSystemPath(buf.as_mut_ptr()) };
    // SAFETY: c_char and u8 have the same layout.
    let bytes = unsafe { std::slice::from_raw_parts(buf.as_ptr().cast::<u8>(), buf.len()) };
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    PathBuf::from(String::from_utf8_lossy(&bytes[..end]).into_owned())
}

pub fn screen_bounds() -> (i32, i32, i32, i32) {
    let (mut l, mut t, mut r, mut b) = (0, 0, 0, 0);
    unsafe { XPLMGetScreenBoundsGlobal(&mut l, &mut t, &mut r, &mut b) };
    (l, t, r, b)
}

#[derive(Clone, Copy)]
pub struct DataRef(XPLMDataRef);

impl DataRef {
    pub fn find(name: &str) -> Option<Self> {
        let s = cstring(name);
        let r = unsafe { XPLMFindDataRef(s.as_ptr()) };
        (!r.is_null()).then_some(Self(r))
    }

    pub fn f32(self) -> f32 {
        unsafe { XPLMGetDataf(self.0) }
    }

    pub fn f64(self) -> f64 {
        unsafe { XPLMGetDatad(self.0) }
    }

    pub fn i32(self) -> i32 {
        unsafe { XPLMGetDatai(self.0) }
    }

    /// The first `n` elements of a float array dataref.
    pub fn f32s(self, n: usize) -> Vec<f32> {
        let mut out = vec![0.0f32; n];
        let got = unsafe { XPLMGetDatavf(self.0, out.as_mut_ptr(), 0, n as c_int) };
        out.truncate(got.max(0) as usize);
        out
    }

    /// The first `n` elements of an int array dataref.
    pub fn i32s(self, n: usize) -> Vec<i32> {
        let mut out = vec![0 as c_int; n];
        let got = unsafe { XPLMGetDatavi(self.0, out.as_mut_ptr(), 0, n as c_int) };
        out.truncate(got.max(0) as usize);
        out
    }
}

/// Runs an X-Plane command once, by name. Returns false if it doesn't exist.
pub fn command_once(name: &str) -> bool {
    let s = cstring(name);
    let r = unsafe { XPLMFindCommand(s.as_ptr()) };
    if r.is_null() {
        return false;
    }
    unsafe { XPLMCommandOnce(r) };
    true
}

/// Whether some plugin has X-Plane's camera right now.
pub fn camera_controlled() -> bool {
    let mut how_long: c_int = 0;
    unsafe { XPLMIsCameraBeingControlled(&mut how_long) != 0 }
}

/// A command handler we registered, removed on drop: either on a command
/// this plugin owns, or on one of X-Plane's that we only watch.
pub struct OwnedCommand {
    cmd: XPLMCommandRef,
    handler: XPLMCommandCallback_f,
    refcon: usize,
}

impl OwnedCommand {
    pub fn new(name: &str, desc: &str, handler: XPLMCommandCallback_f, refcon: usize) -> Self {
        let (n, d) = (cstring(name), cstring(desc));
        unsafe {
            let cmd = XPLMCreateCommand(n.as_ptr(), d.as_ptr());
            XPLMRegisterCommandHandler(cmd, handler, 1, refcon as *mut _);
            Self { cmd, handler, refcon }
        }
    }

    /// Watches an existing X-Plane command. The handler must return 1 so
    /// X-Plane still runs the command itself.
    pub fn watch(name: &str, handler: XPLMCommandCallback_f, refcon: usize) -> Option<Self> {
        let n = cstring(name);
        unsafe {
            let cmd = XPLMFindCommand(n.as_ptr());
            if cmd.is_null() {
                return None;
            }
            XPLMRegisterCommandHandler(cmd, handler, 1, refcon as *mut _);
            Some(Self { cmd, handler, refcon })
        }
    }
}

impl Drop for OwnedCommand {
    fn drop(&mut self) {
        unsafe { XPLMUnregisterCommandHandler(self.cmd, self.handler, 1, self.refcon as *mut _) }
    }
}

/// A menu we created, with its item in the parent menu.
pub struct Menu {
    id: XPLMMenuID,
    parent: XPLMMenuID,
    parent_item: c_int,
}

impl Menu {
    /// A submenu under X-Plane's Plugins menu.
    pub fn in_plugins_menu(title: &str, handler: XPLMMenuHandler_f) -> Option<Self> {
        Self::create(unsafe { XPLMFindPluginsMenu() }, title, handler)
    }

    fn create(parent: XPLMMenuID, title: &str, handler: XPLMMenuHandler_f) -> Option<Self> {
        let t = cstring(title);
        unsafe {
            let parent_item = XPLMAppendMenuItem(parent, t.as_ptr(), ptr::null_mut(), 0);
            if parent_item < 0 {
                return None;
            }
            let id = XPLMCreateMenu(t.as_ptr(), parent, parent_item, Some(handler), ptr::null_mut());
            (!id.is_null()).then_some(Self { id, parent, parent_item })
        }
    }

    /// Appends an item and returns its index, for [`Menu::set_checked`].
    pub fn item(&self, name: &str, item_ref: usize) -> c_int {
        let n = cstring(name);
        unsafe { XPLMAppendMenuItem(self.id, n.as_ptr(), item_ref as *mut _, 0) }
    }

    pub fn separator(&self) {
        unsafe { XPLMAppendMenuSeparator(self.id) }
    }

    pub fn set_checked(&self, index: c_int, checked: bool) {
        if index >= 0 {
            let state = if checked { xplm_Menu_Checked } else { xplm_Menu_Unchecked };
            unsafe { XPLMCheckMenuItem(self.id, index, state) }
        }
    }
}

impl Drop for Menu {
    fn drop(&mut self) {
        unsafe {
            XPLMDestroyMenu(self.id);
            XPLMRemoveMenuItem(self.parent, self.parent_item);
        }
    }
}

/// A Y-probe for terrain height under a point.
pub struct Probe(XPLMProbeRef);

impl Probe {
    pub fn new() -> Option<Self> {
        let p = unsafe { XPLMCreateProbe(xplm_ProbeY) };
        (!p.is_null()).then_some(Self(p))
    }

    /// Terrain height (local y, meters) under `x, z`, probing from `y`.
    pub fn ground_y(&self, x: f64, y: f64, z: f64) -> Option<f64> {
        let mut info = XPLMProbeInfo_t { structSize: std::mem::size_of::<XPLMProbeInfo_t>() as c_int, ..Default::default() };
        let hit = unsafe { XPLMProbeTerrainXYZ(self.0, x as f32, y as f32, z as f32, &mut info) };
        (hit == xplm_ProbeHitTerrain).then_some(info.locationY as f64)
    }
}

impl Drop for Probe {
    fn drop(&mut self) {
        unsafe { XPLMDestroyProbe(self.0) }
    }
}
