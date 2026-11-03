# Camship

A cinematic camera plugin for X-Plane 11.50+/12, written in Rust. It films your aircraft the way a camera ship and a ground crew would for a commercial. It cuts between moving outside shots: formation slides, overtakes, orbits, head-on and top-down views, a wingtip camera, engine close-ups and flybys.

## Installing and updating

Download `Camship-vX.Y.Z.zip` from the [latest release](https://github.com/emergencybutter/camship/releases/latest) and copy the `Camship` folder into `X-Plane/Resources/plugins/`. It works on Windows, macOS (Intel and Apple Silicon) and Linux.

**Updating through [SkunkCrafts Updater](https://forums.x-plane.org/forums/topic/292710-20250206-skunkcrafts-updater-standalone-client-v32d-available/) is recommended.** The plugin folder includes a `skunkcrafts_updater.cfg`, so the updater picks up Camship on its own and fetches each new release. Without it, quit X-Plane and replace the `Camship` folder. Your settings are kept either way.

### Building from source

```
cargo xtask dist                                # builds dist/Camship/64/win.xpl (or lin/mac)
cargo xtask dist --install "D:\X-Plane 12"      # also copies it into Resources/plugins
```

CI builds Windows, Linux and a universal (Intel + Apple Silicon) macOS binary on every push. Pushing a `vX.Y.Z` tag attaches the all-platform zip to a GitHub release and publishes it to the `skunkcrafts` branch, which the updater reads.

## Using it

**Plugins → Camship**

| Item | |
|---|---|
| **Cinematic camera** | Switches to an outside view and starts the director. Select it again to return to your previous view (3D cockpit, 2D panel or HUD). Choosing any other X-Plane view also ends it. |
| **Next shot** | Cuts to the next shot straight away. If Camship is off, this starts it. |
| **Settings...** | Opens the Camship window. |

**The Camship window** is a floating X-Plane window that follows X-Plane's UI scale, and you can pop it out.

- **Header:** Start/Stop, the shot on screen, **Next shot**, and **Hold** (the move plays back and forth instead of ending).
- **Shots:** every shot, with a tick box and a one-line description, in three groups: *Camera plane*, *Close-ups* and *Special cameras*. Each group, and the whole list, has **All**/**None** buttons. The **play** button cuts to that shot straight away, even if it's unticked, so you can preview it. Unticking the shot on screen moves on to the next one. Engine shots are greyed out on aircraft that don't report engine positions.
- **Camera:** take length, distance, zoom, turn lag, and cut or glide between shots (with the glide time). **Defaults** resets these but keeps your shot list.

Changes apply immediately and are saved by themselves.

You can bind these commands to keys: `camship/toggle`, `camship/next_shot`, `camship/hold`, `camship/toggle_window`.

X-Plane's screenshot (Shift+Space) and movie recording keep working while Camship runs, and they capture Camship's view. If X-Plane takes the camera back within 2 s of one, Camship takes it straight back and carries on with the same shot.

### The shots

All distances scale with the aircraft's size, so the same shot frames a Cessna and a 747. Engine shots use the engine positions the aircraft reports (`sim/flightmodel2/engines/location_*_mtr`), film each engine from its outboard side, and scale with nacelle size. They're skipped for aircraft that don't report engine positions; `Log.txt` lists what was found. Most shots are played from either side at random.

| Shot | Where | Move |
|---|---|---|
| Rear-quarter reveal | air, ground | slides forward from behind the wing to abeam, zoom creeps in |
| Formation abeam | air | camera ship slides from aft of the wing to ahead of it, telephoto |
| Hero front quarter | air, ground | low, ahead of the nose, slight Dutch tilt, push-in |
| Head-on | air, ground | flies ahead of the aircraft looking back at the nose, slowly letting it catch up |
| Overtake across the nose | air, ground | passes the aircraft from behind one wing and crosses slowly ahead of the nose |
| Overtake above | air (high) | passes slowly over the aircraft from behind to ahead |
| Top-down | air, ground | directly above, looking straight down with the aircraft centred and its nose at the top, slowly rising |
| Overhead pass | air (high) | top-down, running nose to tail |
| Slow orbit | air, ground | level 120° arc around the aircraft |
| Under the wing | air (high) | looks up at the belly from below and behind |
| Wingtip cam | air, ground | Insta360-style: an ultra-wide camera on a stick a foot or two ahead of a wingtip, looking back in at the fuselage. It's rigid, so it rolls and pitches with the aircraft |
| Tail detail | air, ground | tight telephoto on the fin |
| Wing detail | air, ground | tight telephoto on the inboard wing |
| Engine: intake | air, ground | close on one engine from ahead and low, easing in on the intake or propeller |
| Engine: nacelle side | air, ground | outboard of the engine, under the wing, sliding front to back |
| Engine: exhaust | air, ground | behind and below an engine, looking forward (engines off the centreline only) |
| Engine: from below | air | looking up at the nacelle |
| Long-lens chase | air, ground | far behind, ~4.5× zoom, compressed perspective |
| Crane up from behind | air, ground | rises from tail height to a high rear view |
| Flyby | air, ground (≥ 25 m/s) | fixed photographer ahead of the aircraft and beside its track (at eye height on the ground), panning as it passes |

What makes it feel filmed rather than computed:

- **Formation lag.** The camera ship holds its position in a frame that follows the aircraft's attitude about 1.2 s late. In a turn or pitch change the aircraft visibly rotates in the frame, and the horizon partly banks with it.
- **Stabilized camera.** No shake or jitter: a critically damped spring gives a smooth, gimbal-steady pan. The aim leads the aircraft by how fast it moves across the view, so it stays centred on a 190°/s flyby or while the camera sweeps overhead.
- **Eased moves and zoom creep.** Every take starts and stops its dolly gently.
- **Safety.** The camera never goes below terrain (probed every frame) plus 2 m. It also stays at least half a wingspan from the aircraft's reference point. Belly and overhead shots are only picked when the aircraft is high enough. Shots that stop fitting, for example a belly shot on short final, are cut early.
- **Variety.** Shots are picked at random by weight. The last three shots are never repeated.

### Preferences

`X-Plane/Output/preferences/camship.prf` holds these settings. The window covers everything except `frame_roll`, `ground_clearance_m` and `subject_size_m`. Edit the file while X-Plane is not running.

| Key | Default | |
|---|---|---|
| `shot_min_s` / `shot_max_s` | 6 / 10 | length of each take, seconds |
| `glide`, `glide_s` | 0, 2 | glide transitions and their length |
| `distance_scale` | 1.5 | how far the cameras sit from the aircraft (0.5–5); raise it to pull back, lower it to get closer |
| `formation_lag_s` | 1.2 | how late the camera ship follows manoeuvres |
| `frame_roll` | 0.5 | how much the formation position banks with the aircraft (0–1) |
| `ground_clearance_m` | 2 | minimum camera height above terrain |
| `subject_size_m` | 0 | aircraft size; 0 = read from `sim/aircraft/view/acf_size_x/z` |
| `zoom_scale` | 1.0 | multiplies every zoom; lower it if your FOV is narrow |
| `disabled_shots` | | comma-separated ids, e.g. `belly,top_down` |

The aircraft size in use is written to `Log.txt` when Camship starts and each time an aircraft loads.

## Design

```
crates/core      camship-core: pure Rust, no X-Plane. Unit-tested with `cargo test`.
  math.rs        Vec3, body→world rotation in X-Plane's local frame, look angles,
                 attitude low-pass, critically damped spring, smooth noise, RNG
  shots.rs       the shot catalog (paths, aim points, zoom, Dutch angle, constraints)
  director.rs    shot picking, takes, glide blending, clearance, operator aim
  settings.rs    prefs + text format
crates/plugin    camship (cdylib → win/lin/mac.xpl)
  ffi.rs         hand-written XPLM bindings (raw-dylib on Windows: no SDK download)
  xp.rs          safe wrappers: datarefs, commands, menus, terrain probe
  gui.rs, gl.rs  egui hosted in an XPLM floating window (shared with PauseAtToD)
  view.rs        the Camship window's content (plain egui code)
  app.rs         state machine: Off → Starting (switch to chase view) → Running
  lib.rs         XPlugin* entry points, camera/menu/command callbacks
xtask            `cargo xtask dist` packaging/installing
```

Camship takes the camera with `XPLMControlCamera(xplm_ControlCameraUntilViewChanges)` one frame after it switches to `sim/view/chase`. This makes X-Plane draw the exterior model, and any view key the user presses hands control back cleanly. Every callback catches panics. If the camera callback panics, it gives the camera back to X-Plane.
