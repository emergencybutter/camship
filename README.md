# Camship

A cinematic camera plugin for X-Plane 11.50+/12, written in Rust. It films your aircraft the way a camera ship and a ground crew would for a commercial. It cuts between moving outside shots: formation slides, overtakes, orbits, a dolly zoom, head-on and top-down views, wingtip, fin-top and belly cameras, a drone reveal, a sun silhouette, engine close-ups and flybys.

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
| **Cinematic camera** | Switches to an outside view and starts the director. Select it again to return to your previous view (3D cockpit, 2D panel or HUD). Choosing any other X-Plane view also ends it. If another plugin takes the camera without a view change (some aircraft plugins do), Camship takes it back and carries on, up to 3 times a minute, and `Log.txt` says what happened. |
| **Next shot** | Cuts to the next shot straight away. If Camship is off, this starts it. |
| **Settings...** | Opens the Camship window. |

**The Camship window** is a floating X-Plane window that follows X-Plane's UI scale, and you can pop it out.

- **Header:** Start/Stop, the shot on screen, **Next shot**, and **Hold** (the move plays back and forth instead of ending).
- **Shots:** every shot, with a tick box and a one-line description, in four groups: *Camera plane*, *Close-ups*, *Special cameras* and *Action shots*. Each group, and the whole list, has **All**/**None** buttons. The **play** button cuts to that shot straight away, even if it's unticked, so you can preview it. Unticking the shot on screen moves on to the next one. Engine shots are greyed out on aircraft that don't report engine positions.
- **Camera:** take length, distance, zoom, turn lag, and the pause: how long the camera holds still at the start of each shot before it moves (with Hold on, it also pauses at each end), and whether to keep filming while the sim is paused. **Defaults** resets these but keeps your shot list.

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
| Front orbit | air, ground | level 120° arc round the front, from ahead of one wing, across the nose, to ahead of the other |
| Sun silhouette | air, ground (sun 1° below to 25° above the horizon) | telephoto from the far side of the sun, swinging slowly so the sun slides behind the aircraft |
| Under the wing | air (high) | looks up at the belly from below and behind |
| Wingtip cam | air, ground | Insta360-style: an ultra-wide camera on a stick a foot or two ahead of a wingtip, looking back in at the fuselage. It's rigid, so it rolls and pitches with the aircraft |
| Fin-top cam | air, ground | rigid ultra-wide camera on a short mast above the fin, looking forward along the whole fuselage, like an airliner's tail camera |
| Belly cam | air, ground | rigid ultra-wide camera under the nose, looking forward and down at the runway |
| Over the shoulder | air, ground | rigid camera above the wing root, looking forward along the nose to the horizon |
| Tail detail | air, ground | tight telephoto on the fin |
| Wing detail | air, ground | tight telephoto on the inboard wing |
| Engine: intake | air, ground | close on one engine from ahead and low, easing in on the intake or propeller |
| Engine: nacelle side | air, ground | outboard of the engine, under the wing, sliding front to back |
| Engine: exhaust | air, ground | behind and below an engine, looking forward (engines off the centreline only) |
| Engine: from below | air | looking up at the nacelle |
| Long-lens chase | air, ground | far behind, ~4.5× zoom, compressed perspective |
| Crane up from behind | air, ground | rises from tail height to a high rear view |
| Dolly zoom | air, ground | backs away to three times the distance while zooming in to match: the aircraft stays the same size and the background swells behind it |
| Breakaway | air | from formation off the wing, the camera ship drops back and down while the aircraft pulls away |
| Flyby | air, ground (≥ 25 m/s) | fixed photographer ahead of the aircraft and beside its track (at eye height on the ground), panning as it passes |
| Planespotter | air, ground (≥ 25 m/s) | like Flyby, but far off the track on a ~4× long lens, so the pan is slow and the background compressed |
| Under the approach | air, low (≤ 4 sizes up, ≥ 25 m/s) | standing on the ground almost right under the flight path, looking up as the aircraft passes overhead |
| Drone reveal | air, ground | a rising drone ahead and to the side, its lens tilting up from the scenery until it finds the aircraft |
| Locked-off frame | air, ground (≥ 25 m/s) | a camera on a tripod beside the track that never pans: the aircraft enters the frame, crosses it and leaves |

### Action shots

Action shots are never picked at random. When one of these things happens, Camship cuts straight to one of its shots, chosen at random among the ticked ones, and lets the moment play out (4 s, or until just after contact for a touchdown) before a less important action takes over; that one waits its turn. A more important action cuts in after half a second, so a landing goes touchdown, spoilers, reversers. If two happen together, the one that's over sooner wins (a touchdown beats the reversers, which beat the flaps). With **Hold** on, it stays on your shot. Untick **Action shots** in the group header to switch them all off (`action_shots 0` in `camship.prf`).

| Action | Detected when | Shots |
|---|---|---|
| Gear | the gear starts to extend or retract (`sim/flightmodel2/gear/deploy_ratio`) | *from below* (under the wing, looking up at a main gear), *nose gear* (ahead and below it), *wheel-well cam* (rigid, under the wing, looking back into the bay) |
| Flaps | the flaps start to move, at each stage (`flap1_deploy_ratio`) | *window seat* (a passenger's view of the inboard flap), *trailing edge* (from behind and above the wing) |
| Touchdown | the wheels are about 2.5 s from the runway at the current sink rate (predicted, so the camera is in place before they touch) | *runway edge* (a photographer by the runway at the touchdown point, tight on the main gear), *gear cam* (rigid, beside the main wheels), *low chase* (low behind the main gear) |
| Reversers | a thrust reverser opens (`thrust_reverser_deploy_ratio`) | *nacelle* (outboard of the engine), *cabin window* (from the fuselage, looking out at the engine) |
| Takeoff roll | accelerating harder than 1.2 m/s² past 12 m/s on the ground, after being nearly stopped | *runway edge* (long lens from the edge ahead, the aircraft charging past), *from behind* (low, far behind, long lens) |
| Rotation | on the ground above 20 m/s, still accelerating, the nose coming up faster than 1.5°/s (also on a touch-and-go) | *runway edge* (the aircraft lifts off as it passes), *gear cam* (the wheels leaving the runway), *chase* (low behind as it pulls up) |
| Ground spoilers | the spoilers go past half on the ground (`speedbrake_ratio`) | *window seat* (looking out over the wing), *above the wing* |
| Go-around | climbing faster than 2.5 m/s for a second, low, within 20 s of descending close to the ground | *climb-out* (long lens from behind), *under the climb* (standing under the flight path) |
| Steep turn | banked past 45° for a second | *high outside* (above the high wing, horizon level), *down the low wing* (rigid on the fin, looking down the low wing at the ground) |
| Engine start | an engine starts burning fuel (`engine_is_burning_fuel`) | *intake* and *cabin window*, both on the engine that started |
| Landing lights | the landing lights come on, with the sun under 8° (dusk, night) | *head-on* (looking into the lights), *front quarter* |
| Doors | a door or the canopy opens while parked (`door_open_ratio`, `canopy_open_ratio`) | *cabin door* (beside the forward fuselage, left side) |
| Pushback | rolling backwards for a second | *from the ramp* (high, ahead and to the side) |
| Shutdown | the last engine stops, parked | *pull back* (a 10 s pull back and up to end the flight) |

Gear shots use the wheel positions the aircraft reports (`sim/flightmodel/parts/tire_*_no_deflection`), and a typical layout if it doesn't report any. The nose gear shot is skipped on taildraggers. Reverser and engine-start shots need engine positions. Some add-on aircraft animate their doors with their own datarefs, which Camship can't see. `Log.txt` lists each action and the shot that filmed it, or why it was missed.

What makes it feel filmed rather than computed:

- **Formation lag.** The camera ship holds its position in a frame that follows the aircraft's attitude about 1.2 s late. In a turn or pitch change the aircraft visibly rotates in the frame, and the horizon partly banks with it.
- **Stabilized camera.** No shake or jitter: a critically damped spring gives a smooth, gimbal-steady pan. The aim leads the aircraft by how fast it moves across the view, so it stays centred on a 190°/s flyby or while the camera sweeps overhead.
- **Eased moves and zoom creep.** Every take starts and stops its dolly gently.
- **Safety.** The camera never goes below terrain (probed every frame) plus 2 m. It also stays at least half a wingspan from the aircraft's reference point. Cameras mounted on the airframe are the exception, but they still never go within 0.3 m of the ground. Under-the-wing and overhead shots are only picked when the aircraft is high enough, and Under the approach only when it is low. Shots that stop fitting, for example an under-the-wing shot on short final, are cut early.
- **Variety.** Shots are picked at random by weight. The last three shots are never repeated.
- **Bullet time.** Pause X-Plane and the camera keeps flying around the frozen aircraft, cutting between shots as usual. Untick *Keep filming* to freeze it with the sim instead.
- **Golden hour.** Shots that work from either side pick one at random, but three times out of four they take the side that puts the sun behind the aircraft, so sunrises and sunsets end up in the frame. This only applies when the sun is between 6° below the horizon and 60° up; at night, or with the sun high overhead, either side is equally likely.

### Preferences

`X-Plane/Output/preferences/camship.prf` holds these settings. The window covers everything except `frame_roll`, `ground_clearance_m` and `subject_size_m`. Edit the file while X-Plane is not running.

| Key | Default | |
|---|---|---|
| `shot_min_s` / `shot_max_s` | 6 / 10 | length of each take, seconds |
| `pause_s` | 1 | how long the camera holds still at the start of each shot, seconds (0–5, at most half the take) |
| `distance_scale` | 1.5 | how far the cameras sit from the aircraft (0.5–5); raise it to pull back, lower it to get closer |
| `formation_lag_s` | 1.2 | how late the camera ship follows manoeuvres |
| `frame_roll` | 0.5 | how much the formation position banks with the aircraft (0–1) |
| `ground_clearance_m` | 2 | minimum camera height above terrain |
| `subject_size_m` | 0 | aircraft size; 0 = read from `sim/aircraft/view/acf_size_x/z` |
| `zoom_scale` | 1.0 | multiplies every zoom; lower it if your FOV is narrow |
| `film_when_paused` | 1 | keep the camera moving around the frozen aircraft while the sim is paused |
| `disabled_shots` | | comma-separated ids, e.g. `belly,top_down` |

The aircraft size in use is written to `Log.txt` when Camship starts and each time an aircraft loads.

## Design

```
crates/core      camship-core: pure Rust, no X-Plane. Unit-tested with `cargo test`.
  math.rs        Vec3, body→world rotation in X-Plane's local frame, look angles,
                 attitude low-pass, critically damped spring, smooth noise, RNG
  shots.rs       the shot catalog (paths, aim points, zoom, Dutch angle, constraints)
  director.rs    shot picking, takes, pauses, clearance, operator aim
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
