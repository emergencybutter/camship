//! The director: picks shots, runs each take, and turns it into a camera pose
//! every frame.
//!
//! Per frame:
//! 1. The camera ship's frame follows the subject's attitude through a low-pass
//!    (formation lag), so in a turn the aircraft visibly rotates in the frame.
//! 2. The current take's path gives a camera position in that frame and an aim
//!    point on the airframe; both move with eased timing.
//! 3. Each take opens on a still frame for the pause, then the move starts.
//! 4. The position is pushed out of the airframe and above the terrain.
//! 5. The aim goes through a critically damped spring: a smooth, fully
//!    stabilized pan with no hand-held jitter.

use std::collections::VecDeque;

use crate::events::{Detector, Event, Systems, Trigger};
use crate::math::{ease_in_out, exp_alpha, lerp, look_angles, Attitude, Rng, SmoothAttitude, Spring3, Vec3};
use crate::settings::Settings;
use crate::shots::{nacelle_size, wheel_size, Path, Shot, Target, CATALOG};

/// Natural frequency of the aim spring, rad/s. Higher is a tighter pan.
const AIM_OMEGA: f64 = 6.0;
/// A photographer panning on a fast pass tracks much tighter.
const FIXED_AIM_OMEGA: f64 = 20.0;
/// Time constant smoothing the line-of-sight rate used to lead the aim, s.
const LEAD_TAU: f64 = 0.15;
/// How many recent shots are kept out of the next pick.
const RECENT: usize = 3;
/// The camera never gets closer to the subject's reference point than this, in sizes.
const MIN_RANGE: f64 = 0.5;
/// Eye height of a photographer standing beside the runway, meters.
const EYE_HEIGHT_M: f64 = 1.7;
/// How often a two-sided shot is filmed from the side that faces the sun.
const SUN_BIAS: f64 = 0.75;
/// The sun is used for picking a side only from this far below the horizon
/// (twilight still glows) up to this high, degrees.
const SUN_MIN_ELEVATION: f64 = -6.0;
const SUN_MAX_ELEVATION: f64 = 60.0;
/// Below this difference in how much each side faces the sun, either will do.
const SUN_MIN_DIFFERENCE: f64 = 0.15;
/// The sun silhouette is only filmed with the sun this low, degrees: the
/// disc must be in the sky and low enough to sit behind the aircraft.
const SILHOUETTE_MIN_ELEVATION: f64 = -1.0;
const SILHOUETTE_MAX_ELEVATION: f64 = 25.0;
/// A mounted camera never goes closer to the terrain than this, meters.
const MOUNT_CLEARANCE_M: f64 = 0.3;
/// How far through a take a drone reveal has tilted up onto the aircraft.
const REVEAL_SHARE: f64 = 0.6;
/// When the aircraft passes a fixed camera, as a share of the take.
const FLYBY_PASS: f64 = 0.55;
/// A locked-off frame sees the aircraft pass earlier, so it holds on the
/// empty frame after it has gone, rather than waiting for it to arrive.
const LOCKED_OFF_PASS: f64 = 0.4;
/// X-Plane's default horizontal field of view, for when it can't be read.
const DEFAULT_FOV_DEG: f64 = 60.0;
/// An action shot holds at least this long before another action can cut
/// it, s.
const ACTION_MIN_S: f64 = 4.0;
/// Gear and flaps take a while to move: their shots last at least this long, s.
const ACTION_MIN_TAKE_S: f64 = 6.0;
/// After the wheels touch, the touchdown shot stays on the rollout this long, s.
const ROLLOUT_S: f64 = 3.5;
/// The touchdown shot can't be cut until this long after the wheels touch, s.
const TOUCHDOWN_HOLD_S: f64 = 1.5;
/// An action that couldn't be filmed straight away (another action was on
/// screen) is dropped after this long: the moment has passed, s.
const ACTION_MAX_DELAY_S: f64 = 3.0;
/// A touchdown shot needs this long before the wheels touch to set up, s.
const TOUCHDOWN_MIN_LEAD_S: f64 = 1.0;
/// Where a main wheel sits when the aircraft doesn't say, in sizes
/// (+x right, +y up, +z aft from the reference point).
const GUESS_MAIN_GEAR: Vec3 = Vec3::new(0.12, -0.1, 0.04);
const GUESS_NOSE_GEAR: Vec3 = Vec3::new(0.0, -0.1, -0.32);
/// Wheels within this of the centreline (sizes) are nose or tail wheels.
const CENTRELINE: f64 = 0.04;

/// What the director needs to know about the aircraft, in world coordinates.
#[derive(Clone, Copy, Debug)]
pub struct Subject {
    pub pos: Vec3,
    /// Velocity, m/s.
    pub vel: Vec3,
    pub att: Attitude,
    /// Height above ground, meters.
    pub agl: f64,
    pub on_ground: bool,
    /// Characteristic size (about the wingspan), meters.
    pub size: f64,
    /// Gear, flaps and reversers, for action shots.
    pub systems: Systems,
}

impl Subject {
    pub fn speed(&self) -> f64 {
        self.vel.len()
    }
}

/// Camera placement in X-Plane's terms (degrees; zoom 1 = normal FOV).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraPose {
    pub pos: Vec3,
    pub heading: f64,
    pub pitch: f64,
    pub roll: f64,
    pub zoom: f64,
}

#[derive(Clone, Copy, Debug)]
struct Framing {
    pos: Vec3,
    look: Vec3,
    zoom: f64,
    roll: f64,
}

/// One take of one shot.
#[derive(Clone, Copy, Debug)]
struct Take {
    shot: usize,
    start: f64,
    duration: f64,
    /// +1, or -1 to film from the other side.
    mirror: f64,
    /// World position for shots with a fixed camera.
    anchor: Option<Vec3>,
    /// Where the aircraft will be when it passes a fixed camera.
    pass: Option<Vec3>,
    /// Body-frame position (meters) of the engine or wheel an engine or
    /// gear shot films.
    part: Option<Vec3>,
    /// What an action shot is filming.
    event: Option<Trigger>,
    /// Until when (director clock) no other action may cut this take.
    protect_until: f64,
}

/// An action waiting for the one on screen to finish its moment.
#[derive(Clone, Copy, Debug)]
struct Pending {
    event: Event,
    at: f64,
}

pub struct Director {
    shots: &'static [Shot],
    settings: Settings,
    rng: Rng,
    clock: f64,
    frame: SmoothAttitude,
    take: Option<Take>,
    aim: Spring3,
    /// Previous camera-to-target vector and its smoothed rate, for aim lead.
    sight: Option<(Vec3, Vec3)>,
    recent: VecDeque<usize>,
    hold: bool,
    skip: bool,
    /// A shot the user asked to see next.
    requested: Option<usize>,
    /// Start time of the take the user asked for, so unticking doesn't cut it.
    requested_take: Option<f64>,
    /// Engine positions in the body frame, meters from the reference point.
    engines: Vec<Vec3>,
    /// Wheel positions in the body frame, meters; empty if unknown.
    gear: Vec<Vec3>,
    detector: Detector,
    pending: Option<Pending>,
    /// Lines for the log: what happened and what was filmed.
    notes: Vec<String>,
    /// Unit vector towards the sun, world frame, when it's worth framing.
    sun: Option<Vec3>,
    /// The same, only while the sun is low enough for a silhouette.
    low_sun: Option<Vec3>,
    /// The user's horizontal field of view, degrees (X-Plane's zoom 1).
    fov_deg: f64,
}

impl Director {
    pub fn new(settings: Settings, seed: u64) -> Self {
        Self {
            shots: CATALOG,
            settings,
            rng: Rng::new(seed),
            clock: 0.0,
            frame: SmoothAttitude::default(),
            take: None,
            aim: Spring3::default(),
            sight: None,
            recent: VecDeque::new(),
            hold: false,
            skip: false,
            requested: None,
            requested_take: None,
            engines: Vec::new(),
            gear: Vec::new(),
            detector: Detector::default(),
            pending: None,
            notes: Vec::new(),
            sun: None,
            low_sun: None,
            fov_deg: DEFAULT_FOV_DEG,
        }
    }

    pub fn shots(&self) -> &'static [Shot] {
        self.shots
    }

    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    pub fn settings_mut(&mut self) -> &mut Settings {
        &mut self.settings
    }

    /// Starts over with a fresh first shot (call when the camera is engaged).
    pub fn reset(&mut self) {
        self.frame.reset();
        self.take = None;
        self.aim = Spring3::default();
        self.sight = None;
        self.skip = false;
        self.detector.reset();
        self.pending = None;
    }

    /// What happened since the last call, for the log.
    pub fn take_notes(&mut self) -> Vec<String> {
        std::mem::take(&mut self.notes)
    }

    /// Moves on to the next shot on the next frame.
    pub fn next_shot(&mut self) {
        self.skip = true;
    }

    /// Cuts to shot `index` on the next frame, if it can be filmed now
    /// (enabled or not: this is the user's explicit choice).
    pub fn play(&mut self, index: usize) {
        if index < self.shots.len() {
            self.requested = Some(index);
            self.skip = true;
        }
    }

    /// Whether shot `index` could be filmed right now (ignoring whether it's
    /// enabled), e.g. to grey out engine shots on an aircraft without engine data.
    pub fn available(&self, index: usize, s: &Subject) -> bool {
        self.shots.get(index).is_some_and(|shot| self.fits(shot, s))
    }

    /// Whether shot `index` has the data it needs for this aircraft at all,
    /// whatever the aircraft is doing.
    pub fn supported(&self, index: usize, size: f64) -> bool {
        self.shots.get(index).is_some_and(|shot| shot.target == Target::Airframe || !self.parts_for(shot, size).is_empty())
    }

    /// Wheel positions (body frame, meters; +x right, +y up, +z aft), or
    /// empty to use a typical layout.
    pub fn set_gear(&mut self, gear: Vec<Vec3>) {
        self.gear = gear;
    }

    /// Height of the reference point above the ground with the wheels on
    /// it, meters, until it can be measured on the ground.
    fn rest_height(&self, size: f64) -> f64 {
        let lowest = self.gear.iter().map(|g| g.y).fold(f64::INFINITY, f64::min);
        let centre = if lowest.is_finite() && lowest < 0.0 { -lowest } else { -GUESS_MAIN_GEAR.y * size };
        // The wheel centre plus about a wheel radius.
        centre + 0.5 * wheel_size(size)
    }

    /// Engine positions (body frame, meters; +x right, +y up, +z aft).
    /// Engine shots are only picked when this is not empty.
    pub fn set_engines(&mut self, engines: Vec<Vec3>) {
        self.engines = engines;
    }

    /// The user's horizontal field of view in degrees, so shots with a fixed
    /// lens (`fov_deg`) get the same picture whatever it's set to.
    pub fn set_fov(&mut self, fov_deg: f64) {
        self.fov_deg = if fov_deg.is_finite() && (10.0..=170.0).contains(&fov_deg) { fov_deg } else { DEFAULT_FOV_DEG };
    }

    /// The X-Plane zoom for a shot at progress `e`.
    fn zoom(&self, shot: &Shot, e: f64) -> f64 {
        let zoom = match shot.fov_deg {
            // Zoom scales the focal length: tan(fov / 2) shrinks by 1 / zoom.
            Some(fov) => (self.fov_deg / 2.0).to_radians().tan() / (fov.clamp(10.0, 170.0) / 2.0).to_radians().tan(),
            None => lerp(shot.zoom.0, shot.zoom.1, e),
        };
        (zoom * self.settings.zoom_scale).max(0.1)
    }

    /// Where the sun is: true heading and elevation, degrees. Two-sided shots
    /// then favour the side that puts the sun behind the aircraft, so sunrises
    /// and sunsets end up in the frame.
    pub fn set_sun(&mut self, heading: f64, elevation: f64) {
        let dir = (heading.is_finite() && elevation.is_finite()).then(|| {
            let (h, p) = (heading.to_radians(), elevation.to_radians());
            Vec3::new(h.sin() * p.cos(), p.sin(), -h.cos() * p.cos())
        });
        self.sun = dir.filter(|_| (SUN_MIN_ELEVATION..=SUN_MAX_ELEVATION).contains(&elevation));
        self.low_sun = dir.filter(|_| (SILHOUETTE_MIN_ELEVATION..=SILHOUETTE_MAX_ELEVATION).contains(&elevation));
    }

    /// The engines or wheels a shot could film.
    fn parts_for(&self, shot: &Shot, size: f64) -> Vec<Vec3> {
        match shot.target {
            Target::Airframe => Vec::new(),
            Target::Engine { outboard_only } => {
                self.engines.iter().copied().filter(|e| !outboard_only || e.x.abs() > 0.1 * size).collect()
            }
            Target::Gear { nose } if self.gear.is_empty() => {
                if nose {
                    vec![GUESS_NOSE_GEAR * size]
                } else {
                    vec![GUESS_MAIN_GEAR * size, GUESS_MAIN_GEAR.mirror_x(-1.0) * size]
                }
            }
            // The most forward centreline wheel, if it's ahead (not a tailwheel).
            Target::Gear { nose: true } => self
                .gear
                .iter()
                .copied()
                .filter(|g| g.x.abs() <= CENTRELINE * size && g.z < 0.0)
                .min_by(|a, b| a.z.total_cmp(&b.z))
                .into_iter()
                .collect(),
            Target::Gear { nose: false } => self.gear.iter().copied().filter(|g| g.x.abs() > CENTRELINE * size).collect(),
        }
    }

    /// Where a take's offsets are measured from (body frame, meters) and
    /// their unit, meters: the airframe in sizes, or an engine or wheel.
    fn origin(&self, take: &Take, size: f64) -> (Vec3, f64) {
        match (take.part, self.shots[take.shot].target) {
            (Some(p), Target::Engine { .. }) => (p, nacelle_size(size)),
            (Some(p), Target::Gear { .. }) => (p, wheel_size(size)),
            _ => (Vec3::ZERO, size),
        }
    }

    pub fn hold(&self) -> bool {
        self.hold
    }

    /// While held, the current take keeps going back and forth instead of
    /// ending.
    pub fn set_hold(&mut self, hold: bool) {
        if self.hold && !hold {
            // Give the released take a fresh full duration from where it is.
            if let Some(t) = self.take.as_mut() {
                t.start = self.clock;
            }
        }
        self.hold = hold;
    }

    pub fn current_shot(&self) -> Option<&'static Shot> {
        self.take.map(|t| &self.shots[t.shot])
    }

    pub fn update(&mut self, s: &Subject, dt: f64, ground: &mut dyn FnMut(Vec3) -> Option<f64>) -> CameraPose {
        let dt = if dt.is_finite() { dt.clamp(0.0, 0.25) } else { 0.0 };
        self.clock += dt;
        let frame = self.frame.update(s.att, dt, self.settings.formation_lag_s);

        let mut cut = self.action(s, dt, frame, ground);
        let expired = match self.take {
            None => true,
            Some(t) => {
                self.skip
                    || (!self.hold && self.clock - t.start >= t.duration)
                    // Only cut for a misfit when something else fits; otherwise
                    // the fallback shot would be re-picked every frame, flipping
                    // sides at random.
                    || (!self.still_fits(&t, s) && self.any_fit(s))
                    // Unticked while on screen: move on, unless the user asked
                    // for it or nothing else is ticked.
                    || (self.settings.is_disabled(self.shots[t.shot].id)
                        && self.requested_take != Some(t.start)
                        && self.any_enabled_fit(s))
            }
        };
        if expired {
            self.skip = false;
            let wanted = self.requested;
            let shot = self.pick(s);
            self.requested_take = (wanted == Some(shot)).then_some(self.clock);
            let duration = self.duration();
            self.take = Some(self.new_take(shot, s, frame, ground, duration, None, None));
            cut = true;
        }
        let take = self.take.expect("a take was just set");

        match self.shots[take.shot].path {
            // Rigid: no lag, no smoothing, no clearance push. It's bolted on.
            Path::Mount { at, offset_m } => return self.mounted(&take, s, at, offset_m, ground),
            Path::Overhead { .. } => return self.overhead(&take, s, frame),
            Path::LockedOff { .. } => return self.locked_off(&take, s),
            _ => {}
        }

        let mut f = self.framing(&take, s, frame);
        f.pos = self.keep_clear(f.pos, s, ground);
        let tilt = self.shots[take.shot].reveal_tilt;
        if tilt > 0.0 {
            // Drone reveal: aim below the aircraft, tilting up onto it.
            let k = 1.0 - ease_in_out((self.progress(&take) / REVEAL_SHARE).min(1.0));
            let sight = f.look - f.pos;
            let (heading, pitch) = look_angles(sight);
            let down = (pitch - tilt * k).max(-85.0);
            f.look = f.pos + Attitude::new(heading, down, 0.0).rotate(Vec3::FORWARD) * sight.len();
        }

        let forward = s.att.rotate(Vec3::FORWARD);
        let sight = f.look - f.pos;
        let dir = sight.normalized_or(forward);
        let aimed = if cut || !self.aim.is_ready() {
            self.aim.snap(dir);
            self.sight = Some((sight, Vec3::ZERO));
            dir
        } else if dt <= 0.0 {
            // Called twice in one frame: keep the current aim.
            self.aim.update(dir, 0.0, AIM_OMEGA).normalized_or(dir)
        } else {
            // Lead the target by the spring's steady-state lag (2/omega), using
            // how fast the line of sight is moving. The pan stays smooth but
            // keeps the aircraft centred, even when the camera sweeps past it.
            let omega = if self.is_fixed(&take) { FIXED_AIM_OMEGA } else { AIM_OMEGA };
            let rate = match self.sight {
                Some((prev, rate)) => rate.lerp((sight - prev) * (1.0 / dt), exp_alpha(dt, LEAD_TAU)),
                None => Vec3::ZERO,
            };
            self.sight = Some((sight, rate));
            let target = (sight + rate * (2.0 / omega)).normalized_or(dir);
            self.aim.update(target, dt, omega).normalized_or(dir)
        };
        let (heading, pitch) = look_angles(aimed);
        CameraPose {
            pos: f.pos,
            heading,
            pitch: pitch.clamp(-89.9, 89.9),
            roll: f.roll,
            zoom: (f.zoom * self.settings.zoom_scale).max(0.1),
        }
    }

    /// Watches for actions (gear, flaps, touchdown, reversers) and cuts to
    /// an action shot for one. Returns whether it cut.
    fn action(
        &mut self,
        s: &Subject,
        dt: f64,
        frame: Attitude,
        ground: &mut dyn FnMut(Vec3) -> Option<f64>,
    ) -> bool {
        let rest = self.rest_height(s.size);
        if let Some(event) = self.detector.update(s, dt, rest) {
            self.notes.push(format!("{} {}", event.trigger.label(), event.what));
            // Nothing bumps a touchdown that's still waiting.
            if self.pending.is_none_or(|p| p.event.trigger != Trigger::Touchdown || event.trigger == Trigger::Touchdown) {
                self.pending = Some(Pending { event, at: self.clock });
            }
        }
        let Some(p) = self.pending else { return false };
        let waited = self.clock - p.at;
        let in_s = p.event.in_s - waited;
        let stale = match p.event.trigger {
            Trigger::Touchdown => in_s < TOUCHDOWN_MIN_LEAD_S,
            _ => waited > ACTION_MAX_DELAY_S,
        };
        if stale {
            self.pending = None;
            self.notes.push(format!("{}: missed, another action was on screen", p.event.trigger.label()));
            return false;
        }
        // Held: the user is watching this take. Another action's moment: let it play.
        if self.hold || self.take.is_some_and(|t| t.event.is_some() && self.clock < t.protect_until) {
            return false;
        }
        self.pending = None;
        let Some(shot) = self.pick_action(p.event.trigger, s) else {
            return false;
        };
        let (duration, pass_in, protect) = match p.event.trigger {
            Trigger::Touchdown => (in_s + ROLLOUT_S, Some(in_s), in_s + TOUCHDOWN_HOLD_S),
            _ => (self.duration().max(ACTION_MIN_TAKE_S), None, ACTION_MIN_S),
        };
        let mut take = self.new_take(shot, s, frame, ground, duration, pass_in, Some(p.event.trigger));
        take.protect_until = self.clock + protect.min(duration);
        self.take = Some(take);
        self.requested_take = None;
        self.skip = false;
        self.notes.push(format!("{}: filming {}", p.event.trigger.label(), self.shots[shot].name));
        true
    }

    /// An enabled action shot for `trigger` that can be filmed now.
    fn pick_action(&mut self, trigger: Trigger, s: &Subject) -> Option<usize> {
        let pool: Vec<usize> = (0..self.shots.len())
            .filter(|&i| {
                let sh = &self.shots[i];
                sh.trigger == Some(trigger) && !self.settings.is_disabled(sh.id) && self.fits(sh, s)
            })
            .collect();
        self.weighted(&pool)
    }

    /// One of `pool`, at random by weight.
    fn weighted(&mut self, pool: &[usize]) -> Option<usize> {
        let last = *pool.last()?;
        let total: f64 = pool.iter().map(|&i| self.shots[i].weight.max(0.0)).sum();
        let mut r = self.rng.next_f64() * total;
        for &i in pool {
            r -= self.shots[i].weight.max(0.0);
            if r < 0.0 {
                return Some(i);
            }
        }
        Some(last)
    }

    /// A random take length within the user's range, s.
    fn duration(&mut self) -> f64 {
        let (lo, hi) = (self.settings.shot_min_s, self.settings.shot_max_s.max(self.settings.shot_min_s));
        self.rng.range(lo, hi)
    }

    fn is_fixed(&self, take: &Take) -> bool {
        matches!(self.shots[take.shot].path, Path::Flyby { .. } | Path::LockedOff { .. })
    }

    /// A camera on a tripod that never pans: aimed at where the aircraft
    /// will pass.
    fn locked_off(&mut self, take: &Take, s: &Subject) -> CameraPose {
        let shot = &self.shots[take.shot];
        let pos = take.anchor.unwrap_or(s.pos);
        let (heading, pitch) = look_angles(take.pass.unwrap_or(s.pos) - pos);
        self.aim = Spring3::default();
        self.sight = None;
        CameraPose {
            pos,
            heading: heading.rem_euclid(360.0),
            pitch: pitch.clamp(-89.9, 89.9),
            roll: 0.0,
            zoom: self.zoom(shot, ease_in_out(self.progress(take))),
        }
    }

    /// Straight down from above. The aim spring can't handle a vertical
    /// line of sight (its heading is undefined), so the pose is set directly:
    /// centred on the aircraft, nose up in the frame via the lagged heading.
    fn overhead(&mut self, take: &Take, s: &Subject, frame: Attitude) -> CameraPose {
        let shot = &self.shots[take.shot];
        let f = self.framing(take, s, frame);
        self.aim = Spring3::default();
        self.sight = None;
        CameraPose {
            pos: f.pos,
            heading: frame.heading.rem_euclid(360.0),
            pitch: -90.0,
            roll: 0.0,
            zoom: self.zoom(shot, ease_in_out(self.progress(take))),
        }
    }

    /// `at`: mount point from the take's origin, in its units, plus `offset_m` meters.
    fn mounted(
        &mut self,
        take: &Take,
        s: &Subject,
        at: Vec3,
        offset_m: Vec3,
        ground: &mut dyn FnMut(Vec3) -> Option<f64>,
    ) -> CameraPose {
        let shot = &self.shots[take.shot];
        let e = ease_in_out(self.progress(take));
        let m = take.mirror;
        let (origin, unit) = self.origin(take, s.size);
        let mount = origin + (at * unit + offset_m).mirror_x(m);
        let look = origin + (shot.look_from.lerp(shot.look_to, e) * unit).mirror_x(m);
        // Aim in the body frame, then carry it through the aircraft's attitude.
        let (heading, pitch) = look_angles(look - mount);
        let local = Attitude::new(heading, pitch, shot.dutch * m);
        let world = s.att.compose(local);
        // The next shot starts with a cut, so it re-aims from scratch.
        self.aim = Spring3::default();
        self.sight = None;
        let mut pos = s.pos + s.att.rotate(mount);
        // Mount points are estimates: on an aircraft with a lower belly than
        // assumed, don't film from underground.
        if let Some(gy) = ground(pos) {
            pos.y = pos.y.max(gy + MOUNT_CLEARANCE_M);
        }
        CameraPose {
            pos,
            heading: world.heading.rem_euclid(360.0),
            pitch: world.pitch.clamp(-89.9, 89.9),
            roll: world.roll,
            zoom: self.zoom(shot, e),
        }
    }

    fn fits(&self, shot: &Shot, s: &Subject) -> bool {
        // Parked aircraft can report a hair below the ground (the DWSim ATR
        // sits at -0.01 m): that's on the ground, not out of every shot.
        (if s.on_ground { shot.on_ground } else { shot.in_air })
            && s.agl.max(0.0) >= shot.min_agl * s.size
            && s.agl <= shot.max_agl * s.size
            && s.speed() >= shot.min_speed
            && (!matches!(shot.path, Path::SunBehind { .. }) || self.low_sun.is_some())
            && (shot.target == Target::Airframe || !self.parts_for(shot, s.size).is_empty())
    }

    fn still_fits(&self, take: &Take, s: &Subject) -> bool {
        // A fixed camera stays put whatever the aircraft does, until the take
        // ends; an action shot follows its action through (a touchdown goes
        // from the air to the ground).
        self.is_fixed(take) || take.event.is_some() || self.fits(&self.shots[take.shot], s)
    }

    /// Whether the director may pick this shot at random (action shots it never does).
    fn random_fits(&self, sh: &Shot, s: &Subject) -> bool {
        sh.trigger.is_none() && self.fits(sh, s)
    }

    fn any_fit(&self, s: &Subject) -> bool {
        self.shots.iter().any(|sh| self.random_fits(sh, s))
    }

    fn any_enabled_fit(&self, s: &Subject) -> bool {
        self.shots.iter().any(|sh| !self.settings.is_disabled(sh.id) && self.random_fits(sh, s))
    }

    fn pick(&mut self, s: &Subject) -> usize {
        if let Some(i) = self.requested.take() {
            if self.fits(&self.shots[i], s) {
                return i;
            }
        }
        let current = self.take.map(|t| t.shot);
        let fitting: Vec<usize> = (0..self.shots.len()).filter(|&i| self.random_fits(&self.shots[i], s)).collect();
        let enabled: Vec<usize> =
            fitting.iter().copied().filter(|&i| !self.settings.is_disabled(self.shots[i].id)).collect();
        let fresh: Vec<usize> = enabled.iter().copied().filter(|i| !self.recent.contains(i)).collect();
        let not_current: Vec<usize> = enabled.iter().copied().filter(|&i| Some(i) != current).collect();
        let pool = [fresh, not_current, enabled, fitting].into_iter().find(|p| !p.is_empty());
        let Some(pool) = pool else {
            // Nothing fits (e.g. a size/agl edge case): fall back to a shot that works anywhere.
            return self
                .shots
                .iter()
                .position(|sh| sh.on_ground && sh.in_air && sh.min_agl == 0.0 && sh.trigger.is_none())
                .unwrap_or(0);
        };
        let chosen = self.weighted(&pool).unwrap_or(pool[0]);
        self.recent.push_back(chosen);
        while self.recent.len() > RECENT {
            self.recent.pop_front();
        }
        chosen
    }

    /// `pass_in`: for a fixed camera, when the aircraft should pass it, s
    /// (otherwise part-way through the take).
    #[allow(clippy::too_many_arguments)]
    fn new_take(
        &mut self,
        shot: usize,
        s: &Subject,
        frame: Attitude,
        ground: &mut dyn FnMut(Vec3) -> Option<f64>,
        duration: f64,
        pass_in: Option<f64>,
        event: Option<Trigger>,
    ) -> Take {
        let spec = &self.shots[shot];
        let mut two_sided = spec.mirror;
        let candidates = self.parts_for(spec, s.size);
        let mut side = None;
        let part = (!candidates.is_empty()).then(|| {
            let e = candidates[(self.rng.next_u64() % candidates.len() as u64) as usize];
            // Film each engine or wheel from its outboard side; centreline ones from either.
            if e.x.abs() > 0.05 * s.size {
                side = Some(e.x.signum());
                two_sided = false;
            }
            e
        });
        let take = |mirror: f64, ground: &mut dyn FnMut(Vec3) -> Option<f64>| {
            let fixed = self.anchor(shot, s, duration, pass_in, mirror, ground);
            Take {
                shot,
                start: self.clock,
                duration,
                mirror,
                anchor: fixed.map(|f| f.0),
                pass: fixed.map(|f| f.1),
                part,
                event,
                protect_until: self.clock,
            }
        };
        if !two_sided {
            return take(side.unwrap_or(1.0), ground);
        }
        let (right, left) = (take(1.0, ground), take(-1.0, ground));
        let sunward = self.sun.and_then(|sun| {
            let (a, b) = (self.facing(&right, s, frame, sun), self.facing(&left, s, frame, sun));
            ((a - b).abs() >= SUN_MIN_DIFFERENCE).then_some(a > b)
        });
        let p_right = match sunward {
            Some(true) => SUN_BIAS,
            Some(false) => 1.0 - SUN_BIAS,
            None => 0.5,
        };
        if self.rng.next_f64() < p_right {
            right
        } else {
            left
        }
    }

    /// How much a take looks towards the sun over its move, -1..1.
    fn facing(&self, take: &Take, s: &Subject, frame: Attitude, sun: Vec3) -> f64 {
        let samples = [0.2, 0.5, 0.8];
        let total: f64 = samples
            .iter()
            .map(|&u| {
                let at = Take { start: take.start - u * take.duration, ..*take };
                let f = self.framing(&at, s, frame);
                (f.look - f.pos).normalized_or(Vec3::FORWARD).dot(sun)
            })
            .sum();
        total / samples.len() as f64
    }

    /// World position of a fixed camera, for shots that have one, and where
    /// the aircraft will be when it passes.
    fn anchor(
        &self,
        shot: usize,
        s: &Subject,
        duration: f64,
        pass_in: Option<f64>,
        mirror: f64,
        ground: &mut dyn FnMut(Vec3) -> Option<f64>,
    ) -> Option<(Vec3, Vec3)> {
        let (side, height, on_terrain, share) = match self.shots[shot].path {
            Path::Flyby { side, height, on_terrain } => (side, height, on_terrain, FLYBY_PASS),
            Path::LockedOff { side, height } => (side, height, false, LOCKED_OFF_PASS),
            _ => return None,
        };
        let speed = s.speed();
        let fwd = if speed > 1.0 { s.vel * (1.0 / speed) } else { s.att.rotate(Vec3::FORWARD) };
        let flat = fwd.horizontal().normalized_or(Vec3::FORWARD);
        let right = Vec3::new(-flat.z, 0.0, flat.x);
        let pass = s.pos + fwd * (speed * pass_in.unwrap_or(duration * share));
        let mut p = pass + right * (side * s.size * self.settings.distance_scale * mirror);
        if s.on_ground || on_terrain {
            let terrain = ground(p).unwrap_or(s.pos.y - s.agl);
            p.y = terrain + EYE_HEIGHT_M;
        } else {
            p.y += height * s.size;
        }
        Some((p, pass))
    }

    /// The pause at the start of a take: never more than half of it, so
    /// every shot still moves.
    fn pause(&self, take: &Take) -> f64 {
        self.settings.pause_s.clamp(0.0, 0.5 * take.duration)
    }

    /// How far along its move a take is, 0..1. The take opens on a still
    /// frame for the pause, then moves. While held it plays back and forth,
    /// pausing again at each end before moving back over the same shot.
    fn progress(&self, take: &Take) -> f64 {
        let pause = self.pause(take);
        let travel = (take.duration - pause).max(1e-3);
        let t = self.clock - take.start - pause;
        if t <= 0.0 {
            0.0
        } else if t <= travel {
            t / travel
        } else if self.hold && Some(take.shot) == self.take.map(|t| t.shot) {
            // After the first move: pause, back, pause, forward, ...
            let k = (t - travel).rem_euclid(2.0 * (pause + travel));
            if k < pause {
                1.0
            } else if k < pause + travel {
                1.0 - (k - pause) / travel
            } else if k < 2.0 * pause + travel {
                0.0
            } else {
                (k - 2.0 * pause - travel) / travel
            }
        } else {
            1.0
        }
    }

    fn framing(&self, take: &Take, s: &Subject, frame: Attitude) -> Framing {
        let shot = &self.shots[take.shot];
        let e = ease_in_out(self.progress(take));
        let size = s.size;
        // Camera distance only: aim points stay on the airframe.
        let reach = size * self.settings.distance_scale;
        let m = take.mirror;
        let formation = Attitude::new(frame.heading, frame.pitch, frame.roll * self.settings.frame_roll);
        // Engine and gear shots are built around the engine or wheel, in its own size.
        let (origin, look_unit) = self.origin(take, size);
        let reach = if take.part.is_some() { look_unit * self.settings.distance_scale } else { reach };

        let pos = match shot.path {
            Path::Line { from, to } => s.pos + formation.rotate(origin + from.lerp(to, e).mirror_x(m) * reach),
            Path::Orbit { radius, az_from, az_to, height_from, height_to } => {
                let az = (lerp(az_from, az_to, e) * m).to_radians();
                let h = lerp(height_from, height_to, e);
                let off = Vec3::new(radius * az.sin(), h, -radius * az.cos()) * reach;
                s.pos + frame.level().rotate(off)
            }
            Path::Flyby { .. } | Path::LockedOff { .. } => take.anchor.unwrap_or(s.pos),
            Path::SunBehind { distance, sweep_deg } => {
                // From the aircraft, away from the sun; swung about the vertical.
                let away = -self.low_sun.unwrap_or(s.att.rotate(Vec3::FORWARD));
                let swing = Attitude::new(lerp(-sweep_deg, sweep_deg, e) * m, 0.0, 0.0);
                s.pos + swing.rotate(away) * (distance * reach)
            }
            Path::Mount { at, offset_m } => s.pos + s.att.rotate((at * size + offset_m).mirror_x(m)),
            Path::Overhead { height_from, height_to } => s.pos + Vec3::UP * (lerp(height_from, height_to, e) * reach),
        };

        Framing {
            pos,
            look: s.pos + s.att.rotate(origin + shot.look_from.lerp(shot.look_to, e).mirror_x(m) * look_unit),
            zoom: lerp(shot.zoom.0, shot.zoom.1, e),
            roll: shot.dutch * m + shot.bank_follow * frame.roll,
        }
    }

    fn keep_clear(&self, mut pos: Vec3, s: &Subject, ground: &mut dyn FnMut(Vec3) -> Option<f64>) -> Vec3 {
        let min_range = MIN_RANGE * s.size;
        let d = pos - s.pos;
        if d.len() < min_range {
            pos = s.pos + d.normalized_or(Vec3::UP) * min_range;
        }
        if let Some(gy) = ground(pos) {
            let floor = gy + self.settings.ground_clearance_m;
            if pos.y < floor {
                pos.y = floor;
            }
        }
        pos
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 30.0;

    fn flat(_: Vec3) -> Option<f64> {
        Some(0.0)
    }

    fn cruising(t: f64) -> Subject {
        // 120 m/s due east at 1500 m, in a gentle 15° bank.
        Subject {
            pos: Vec3::new(120.0 * t, 1500.0, 0.0),
            vel: Vec3::new(120.0, 0.0, 0.0),
            att: Attitude::new(90.0, 2.0, 15.0),
            agl: 1500.0,
            on_ground: false,
            size: 20.0,
            systems: Systems::default(),
        }
    }

    fn taxiing(t: f64) -> Subject {
        Subject {
            pos: Vec3::new(0.0, 1.5, -8.0 * t),
            vel: Vec3::new(0.0, 0.0, -8.0),
            att: Attitude::new(0.0, 0.0, 0.0),
            agl: 1.5,
            on_ground: true,
            size: 20.0,
            systems: Systems::default(),
        }
    }

    /// Runs the director for `seconds`, returning (subject, pose, shot id) per frame.
    fn run(d: &mut Director, seconds: f64, subject: fn(f64) -> Subject) -> Vec<(Subject, CameraPose, &'static str)> {
        let mut out = Vec::new();
        let mut t = 0.0;
        while t < seconds {
            let s = subject(t);
            let pose = d.update(&s, DT, &mut flat);
            out.push((s, pose, d.current_shot().unwrap().id));
            t += DT;
        }
        out
    }

    #[test]
    fn camera_stays_above_ground_and_off_the_airframe() {
        for seed in 0..8 {
            let mut d = Director::new(Settings::default(), seed);
            for (s, pose, id) in run(&mut d, 120.0, taxiing) {
                // A mounted camera sits on the airframe by design, and only
                // keeps off the ground itself.
                let mounted = matches!(CATALOG.iter().find(|sh| sh.id == id).unwrap().path, Path::Mount { .. });
                let floor = if mounted { MOUNT_CLEARANCE_M } else { 2.0 };
                assert!(pose.pos.y >= floor - 1e-9, "{id} below clearance: {}", pose.pos.y);
                assert!(mounted || (pose.pos - s.pos).len() >= 0.5 * s.size - 1e-6, "{id} inside the aircraft");
            }
        }
    }

    /// Parked a hair below the ground, like the DWSim ATR 72 (y_agl -0.01 m).
    fn parked_below_ground(_: f64) -> Subject {
        Subject { vel: Vec3::ZERO, agl: -0.0116, ..taxiing(0.0) }
    }

    /// Frame-to-frame camera heading jumps over 20°: cuts, or flicker.
    fn heading_jumps(out: &[(Subject, CameraPose, &'static str)]) -> usize {
        out.windows(2)
            .filter(|w| {
                let d = (w[1].1.heading - w[0].1.heading).rem_euclid(360.0);
                d > 20.0 && d < 340.0
            })
            .count()
    }

    #[test]
    fn parked_slightly_below_ground_still_gets_shots() {
        let mut d = Director::new(Settings::default(), 7);
        let out = run(&mut d, 60.0, parked_below_ground);
        let ids: std::collections::HashSet<_> = out.iter().map(|o| o.2).collect();
        assert!(ids.len() > 1, "only {ids:?}");
        // Takes last seconds, not a frame: no flicker between sides.
        let jumps = heading_jumps(&out);
        assert!(jumps < 30, "{jumps} heading jumps in 60 s");
    }

    #[test]
    fn nothing_fitting_holds_the_fallback_instead_of_recutting() {
        // In the air and below ground: no shot fits at all.
        fn nowhere(t: f64) -> Subject {
            Subject { on_ground: false, agl: -50.0, ..taxiing(t) }
        }
        let mut d = Director::new(Settings::default(), 9);
        let jumps = heading_jumps(&run(&mut d, 2.0, nowhere));
        assert!(jumps <= 1, "{jumps} heading jumps in 2 s");
    }

    #[test]
    fn ground_shots_only_on_ground() {
        let mut d = Director::new(Settings::default(), 3);
        for (_, _, id) in run(&mut d, 300.0, taxiing) {
            let shot = CATALOG.iter().find(|s| s.id == id).unwrap();
            assert!(shot.on_ground, "{id} picked while taxiing");
            assert_ne!(id, "flyby", "flyby needs speed");
        }
    }

    #[test]
    fn shots_rotate_without_immediate_repeats() {
        let mut d = Director::new(Settings::default(), 11);
        let mut sequence: Vec<&str> = Vec::new();
        for (_, _, id) in run(&mut d, 600.0, cruising) {
            if sequence.last() != Some(&id) {
                sequence.push(id);
            }
        }
        assert!(sequence.len() >= 50, "only {} takes in 10 min", sequence.len());
        for w in sequence.windows(4) {
            assert!(w[1..].iter().all(|x| *x != w[0]), "repeat within {RECENT}: {w:?}");
        }
        let distinct: std::collections::HashSet<_> = sequence.iter().collect();
        assert!(distinct.len() >= 8, "{distinct:?}");
    }

    #[test]
    fn take_lengths_respect_settings() {
        let settings = Settings { shot_min_s: 4.0, shot_max_s: 5.0, ..Settings::default() };
        let mut d = Director::new(settings, 5);
        let frames = run(&mut d, 120.0, cruising);
        let mut len = 0usize;
        let mut lens = Vec::new();
        for w in frames.windows(2) {
            len += 1;
            if w[0].2 != w[1].2 {
                lens.push(len as f64 * DT);
                len = 0;
            }
        }
        assert!(!lens.is_empty());
        for l in lens {
            assert!((4.0 - 2.0 * DT..=5.0 + 2.0 * DT).contains(&l), "take of {l} s");
        }
    }

    #[test]
    fn disabled_shots_are_skipped() {
        let mut settings = Settings::default();
        for id in ["orbit", "abeam", "flyby", "long_lens"] {
            settings.set_disabled(id, true);
        }
        let mut d = Director::new(settings, 9);
        for (_, _, id) in run(&mut d, 300.0, cruising) {
            assert!(!["orbit", "abeam", "flyby", "long_lens"].contains(&id), "{id}");
        }
    }

    #[test]
    fn hold_keeps_the_shot_and_next_cuts() {
        let mut d = Director::new(Settings::default(), 2);
        let s = cruising(0.0);
        d.update(&s, DT, &mut flat);
        let first = d.current_shot().unwrap().id;
        d.set_hold(true);
        for _ in 0..(60.0 / DT) as usize {
            d.update(&s, DT, &mut flat);
        }
        assert_eq!(d.current_shot().unwrap().id, first);
        d.next_shot();
        d.update(&s, DT, &mut flat);
        assert_ne!(d.current_shot().unwrap().id, first);
    }

    #[test]
    fn takes_open_on_a_still_frame_then_move() {
        let settings = Settings { pause_s: 1.0, ..only("abeam") };
        let mut d = Director::new(settings, 3);
        let s = cruising(0.0);
        // The aircraft holds still, so any camera motion is the move itself.
        let mut poses = Vec::new();
        for _ in 0..(3.0 / DT) as usize {
            poses.push(d.update(&s, DT, &mut flat).pos);
        }
        let still = (0.9 / DT) as usize;
        for p in &poses[1..still] {
            assert!((*p - poses[0]).len() < 1e-6, "moved during the pause");
        }
        assert!((poses[poses.len() - 1] - poses[0]).len() > 0.5, "never moved after the pause");
    }

    #[test]
    fn held_take_pauses_at_each_end() {
        let settings = Settings { pause_s: 1.0, shot_min_s: 6.0, shot_max_s: 6.0, ..Settings::default() };
        let mut d = Director::new(settings, 1);
        d.set_hold(true);
        d.update(&cruising(0.0), 0.0, &mut flat);
        let take = d.take.unwrap();
        let mut at = |t: f64| {
            d.clock = take.start + t;
            d.progress(&take)
        };
        // 1 s still, 5 s forward, 1 s still at the end, 5 s back, 1 s still, forward again.
        assert_eq!(at(0.5), 0.0);
        assert!((at(3.5) - 0.5).abs() < 1e-9);
        assert_eq!(at(6.5), 1.0);
        assert!((at(9.5) - 0.5).abs() < 1e-9);
        assert_eq!(at(12.5), 0.0);
        assert!((at(15.5) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn pause_never_eats_the_whole_take() {
        let settings = Settings { pause_s: 5.0, shot_min_s: 4.0, shot_max_s: 4.0, ..Settings::default() };
        let mut d = Director::new(settings, 1);
        d.update(&cruising(0.0), 0.0, &mut flat);
        let take = d.take.unwrap();
        assert_eq!(d.pause(&take), 2.0);
        d.clock = take.start + 3.0;
        assert!((d.progress(&take) - 0.5).abs() < 1e-9);
    }

    /// Fraction of `abeam` takes filmed looking towards a sun due north,
    /// with the aircraft flying east.
    fn sunward_share(sun: Option<(f64, f64)>) -> f64 {
        let mut d = Director::new(Settings { shot_min_s: 2.0, shot_max_s: 2.0, ..only("abeam") }, 7);
        if let Some((h, e)) = sun {
            d.set_sun(h, e);
        }
        let s = cruising(0.0);
        let (mut toward, mut takes) = (0, 0);
        for _ in 0..400 {
            d.next_shot();
            let pose = d.update(&s, DT, &mut flat);
            takes += 1;
            // Looking north means the camera sits south of the aircraft.
            if pose.pos.z > s.pos.z {
                toward += 1;
            }
        }
        toward as f64 / takes as f64
    }

    #[test]
    fn two_sided_shots_favour_the_sun_behind_the_aircraft() {
        let low_sun = sunward_share(Some((0.0, 5.0)));
        assert!((0.68..=0.82).contains(&low_sun), "{low_sun}");
        // Night, a sun overhead, or no sun at all: either side.
        for sun in [Some((0.0, -30.0)), Some((0.0, 80.0)), None] {
            let share = sunward_share(sun);
            assert!((0.42..=0.58).contains(&share), "{sun:?}: {share}");
        }
    }

    #[test]
    fn wingtip_lens_is_120_degrees_whatever_the_user_fov() {
        for (user_fov, zoom_scale) in [(60.0, 1.0), (90.0, 1.0), (75.0, 1.3)] {
            let mut d = Director::new(Settings { zoom_scale, ..only("wingtip") }, 1);
            d.set_fov(user_fov);
            let pose = d.update(&cruising(0.0), DT, &mut flat);
            let fov = 2.0 * ((user_fov / 2.0_f64).to_radians().tan() / (pose.zoom / zoom_scale)).atan().to_degrees();
            assert!((fov - 120.0).abs() < 1e-6, "user {user_fov}: {fov}");
        }
    }

    #[test]
    fn flyby_camera_is_fixed_and_beside_the_track() {
        let mut settings = Settings::default();
        for s in CATALOG.iter().filter(|s| s.id != "flyby") {
            settings.set_disabled(s.id, true);
        }
        let mut d = Director::new(settings, 1);
        let frames = run(&mut d, 5.0, cruising);
        assert!(frames.iter().all(|f| f.2 == "flyby"));
        let first = frames[0].1.pos;
        let last = frames[frames.len() - 1].1.pos;
        assert!((last - first).len() < 1e-6, "flyby camera moved");
        // Ahead of the aircraft along +x (east), offset sideways in z.
        assert!(first.x > frames[0].0.pos.x + 100.0);
        assert!(first.z.abs() > 1.8 * 1.5 * 20.0 - 1e-6);
    }

    /// Plays only `id`, returning the camera's position along the nose axis
    /// (positive = ahead of the aircraft) and its height above it, per frame.
    fn track(id: &str) -> Vec<(f64, f64)> {
        let mut settings = Settings { formation_lag_s: 0.0, shot_min_s: 8.0, shot_max_s: 8.0, ..Settings::default() };
        for s in CATALOG.iter().filter(|s| s.id != id) {
            settings.set_disabled(s.id, true);
        }
        let mut d = Director::new(settings, 2);
        let s = Subject { att: Attitude::new(90.0, 0.0, 0.0), ..cruising(0.0) };
        let nose = s.att.rotate(Vec3::FORWARD);
        (0..(7.9 / DT) as usize)
            .map(|_| {
                let p = d.update(&s, DT, &mut flat).pos - s.pos;
                assert_eq!(d.current_shot().unwrap().id, id);
                (p.dot(nose), p.y)
            })
            .collect()
    }

    #[test]
    fn wingtip_cam_is_rigid_with_the_airframe() {
        let mut settings = Settings::default();
        for sh in CATALOG.iter().filter(|sh| sh.id != "wingtip") {
            settings.set_disabled(sh.id, true);
        }
        let mut d = Director::new(settings, 3);
        for (i, roll) in [0.0, 30.0, -45.0, 60.0].into_iter().enumerate() {
            let s = Subject { att: Attitude::new(40.0 * i as f64, 5.0, roll), ..cruising(i as f64) };
            let pose = d.update(&s, DT, &mut flat);
            // Back in the body frame, the camera is always at the same spot,
            // looking the same way, upright relative to the wing.
            let att = Attitude::new(pose.heading, pose.pitch, pose.roll);
            let back = |v: Vec3| {
                let b = s.att;
                // Inverse rotation: dot with the body axes.
                let (x, y, z) = (b.rotate(Vec3::new(1.0, 0.0, 0.0)), b.rotate(Vec3::UP), b.rotate(Vec3::new(0.0, 0.0, 1.0)));
                Vec3::new(v.dot(x), v.dot(y), v.dot(z))
            };
            // Half a span out (20 m aircraft), 1.1 m ahead of the tip, a bit up.
            let at = back(pose.pos - s.pos);
            assert!((at.x.abs() - 10.0).abs() < 1e-6, "{at:?}");
            assert!((at.y - 0.7).abs() < 1e-6 && (at.z - -1.5).abs() < 1e-6, "{at:?}");
            let up = back(att.rotate(Vec3::UP));
            assert!(up.y > 0.99, "camera not upright on the wing at roll {roll}: {up:?}");
            let fwd = back(att.rotate(Vec3::FORWARD));
            assert!(fwd.x * at.x < 0.0, "not looking in at the fuselage");
        }
    }

    #[test]
    fn top_down_is_centred_overhead_and_nose_up() {
        let mut settings = Settings { formation_lag_s: 0.0, ..Settings::default() };
        for sh in CATALOG.iter().filter(|sh| sh.id != "straight_down") {
            settings.set_disabled(sh.id, true);
        }
        let mut d = Director::new(settings, 4);
        for subject in [cruising as fn(f64) -> Subject, taxiing] {
            for i in 0..60 {
                let s = subject(i as f64 * DT);
                let pose = d.update(&s, DT, &mut flat);
                assert_eq!(d.current_shot().unwrap().id, "straight_down");
                let off = pose.pos - s.pos;
                assert!(off.horizontal().len() < 1e-9, "not centred: {off:?}");
                assert!(off.y > 1.5 * 20.0, "too low: {}", off.y);
                assert_eq!(pose.pitch, -90.0);
                assert!(crate::math::wrap180(pose.heading - s.att.heading).abs() < 1e-9);
            }
        }
    }

    fn only(id: &str) -> Settings {
        let mut settings = Settings { formation_lag_s: 0.0, ..Settings::default() };
        for sh in CATALOG.iter().filter(|sh| sh.id != id) {
            settings.set_disabled(sh.id, true);
        }
        settings
    }

    const ENGINE_SHOTS: [&str; 4] = ["engine_front", "engine_side", "engine_rear", "engine_below"];

    #[test]
    fn play_cuts_to_the_requested_shot_even_if_unticked() {
        let mut settings = Settings::default();
        settings.set_disabled("orbit", true);
        let mut d = Director::new(settings, 21);
        let s = cruising(0.0);
        d.update(&s, DT, &mut flat);
        let orbit = CATALOG.iter().position(|sh| sh.id == "orbit").unwrap();
        d.play(orbit);
        for _ in 0..30 {
            d.update(&s, DT, &mut flat);
            assert_eq!(d.current_shot().unwrap().id, "orbit");
        }
    }

    #[test]
    fn unticking_the_current_shot_moves_on() {
        let mut d = Director::new(Settings::default(), 22);
        let s = cruising(0.0);
        d.update(&s, DT, &mut flat);
        let id = d.current_shot().unwrap().id;
        d.settings_mut().set_disabled(id, true);
        d.update(&s, DT, &mut flat);
        assert_ne!(d.current_shot().unwrap().id, id);
    }

    #[test]
    fn everything_unticked_still_films_without_flicker() {
        let mut settings = Settings::default();
        for sh in CATALOG {
            settings.set_disabled(sh.id, true);
        }
        let mut d = Director::new(settings, 23);
        let frames = run(&mut d, 20.0, cruising);
        let cuts = frames.windows(2).filter(|w| w[0].2 != w[1].2).count();
        assert!(cuts <= 4, "{cuts} cuts in 20 s");
    }

    #[test]
    fn no_engine_shots_without_engine_positions() {
        let mut d = Director::new(Settings::default(), 12);
        for (_, _, id) in run(&mut d, 600.0, cruising) {
            assert!(!ENGINE_SHOTS.contains(&id), "{id} without engines");
        }
    }

    #[test]
    fn engine_shots_frame_an_engine_from_outboard() {
        // Twin jet: engines under each wing, 6 m out, 1.5 m low, 2 m ahead.
        let engines = vec![Vec3::new(-6.0, -1.5, -2.0), Vec3::new(6.0, -1.5, -2.0)];
        for id in ENGINE_SHOTS {
            for seed in 0..6 {
                let mut d = Director::new(only(id), seed);
                d.set_engines(engines.clone());
                let s = cruising(0.0);
                let pose = d.update(&s, DT, &mut flat);
                assert_eq!(d.current_shot().unwrap().id, id);
                let aim = Attitude::new(pose.heading, pose.pitch, 0.0).rotate(Vec3::FORWARD);
                // Aimed at one of the engines, within a nacelle or so.
                let hit = engines.iter().any(|&e| {
                    let w = s.pos + s.att.rotate(e);
                    let to = w - pose.pos;
                    let miss = (to - aim * to.dot(aim)).len();
                    miss < 2.0 * nacelle_size(s.size)
                });
                assert!(hit, "{id} seed {seed} not aimed at an engine");
                // Camera on the engine's outboard side (or further out).
                let back = |v: Vec3| {
                    let b = s.att;
                    Vec3::new(v.dot(b.rotate(Vec3::new(1.0, 0.0, 0.0))), v.dot(b.rotate(Vec3::UP)), v.dot(b.rotate(Vec3::new(0.0, 0.0, 1.0))))
                };
                let cam = back(pose.pos - s.pos);
                assert!(cam.x.abs() > 6.0, "{id} seed {seed} inboard of the engine: {cam:?}");
            }
        }
    }

    #[test]
    fn exhaust_shot_skips_a_nose_propeller() {
        let mut d = Director::new(only("engine_rear"), 1);
        d.set_engines(vec![Vec3::new(0.0, 0.0, -3.0)]);
        d.update(&cruising(0.0), DT, &mut flat);
        assert_ne!(d.current_shot().unwrap().id, "engine_rear");
    }

    #[test]
    fn head_on_stays_ahead_of_the_nose() {
        assert!(track("head_on").iter().all(|&(ahead, _)| ahead > 2.0 * 20.0));
    }

    #[test]
    fn overtakes_go_from_behind_to_ahead() {
        for id in ["overtake_nose", "overtake_above"] {
            let t = track(id);
            assert!(t[0].0 < 0.0, "{id} starts ahead");
            assert!(t[t.len() - 1].0 > 1.5 * 20.0, "{id} doesn't end well ahead");
        }
        assert!(track("overtake_above").iter().all(|&(_, up)| up > 0.4 * 20.0));
    }

    #[test]
    fn distance_scale_pushes_the_camera_out() {
        let near = Settings { distance_scale: 1.0, ..Settings::default() };
        let far = Settings { distance_scale: 2.0, ..Settings::default() };
        let (mut a, mut b) = (Director::new(near, 8), Director::new(far, 8));
        let s = cruising(0.0);
        for _ in 0..30 {
            let pa = a.update(&s, DT, &mut flat);
            let pb = b.update(&s, DT, &mut flat);
            assert_eq!(a.current_shot().unwrap().id, b.current_shot().unwrap().id);
            let (da, db) = ((pa.pos - s.pos).len(), (pb.pos - s.pos).len());
            assert!(db > 1.9 * da && db < 2.1 * da, "{da} -> {db}");
        }
    }

    #[test]
    fn aims_at_the_aircraft() {
        let mut d = Director::new(Settings::default(), 6);
        for (s, pose, id) in run(&mut d, 60.0, cruising) {
            let aim = Attitude::new(pose.heading, pose.pitch, 0.0).rotate(Vec3::FORWARD);
            let to_subject = (s.pos - pose.pos).normalized_or(aim);
            let off_axis = aim.dot(to_subject).clamp(-1.0, 1.0).acos().to_degrees();
            // Mounted cameras aim where they're bolted (ahead and over the nose,
            // down the runway), a reveal starts off the aircraft on purpose,
            // and a locked-off frame doesn't follow it.
            let shot = CATALOG.iter().find(|sh| sh.id == id).unwrap();
            let mounted = matches!(shot.path, Path::Mount { .. }) && id != "wingtip" && id != "fin_cam";
            if mounted || id == "drone_reveal" || id == "locked_off" {
                continue;
            }
            // Detail shots aim at a part of the airframe, so allow some margin.
            assert!(off_axis < 25.0, "{id}: aircraft {off_axis:.1}° off-axis");
        }
    }

    /// Plays only `id` for a whole take, returning the camera's offset from
    /// the aircraft in the body frame and the pose, per frame.
    fn body_track(id: &str, subject: fn(f64) -> Subject) -> Vec<(Vec3, CameraPose)> {
        let settings = Settings { shot_min_s: 8.0, shot_max_s: 8.0, pause_s: 0.0, ..only(id) };
        let mut d = Director::new(settings, 5);
        (0..(7.9 / DT) as usize)
            .map(|i| {
                let s = subject(i as f64 * DT);
                let pose = d.update(&s, DT, &mut flat);
                assert_eq!(d.current_shot().unwrap().id, id);
                let b = s.att;
                let v = pose.pos - s.pos;
                let body = Vec3::new(
                    v.dot(b.rotate(Vec3::new(1.0, 0.0, 0.0))),
                    v.dot(b.rotate(Vec3::UP)),
                    v.dot(b.rotate(Vec3::new(0.0, 0.0, 1.0))),
                );
                (body, pose)
            })
            .collect()
    }

    fn level(t: f64) -> Subject {
        Subject { att: Attitude::new(90.0, 0.0, 0.0), ..cruising(t) }
    }

    #[test]
    fn dolly_zoom_keeps_the_aircraft_the_same_size() {
        let t = body_track("dolly_zoom", level);
        let (first, last) = (t[0], t[t.len() - 1]);
        assert!(last.0.len() > 2.5 * first.0.len(), "didn't back away: {:?} -> {:?}", first.0, last.0);
        // Apparent size ~ zoom / distance.
        for (body, pose) in &t {
            let ratio = pose.zoom / body.len();
            let start = first.1.zoom / first.0.len();
            assert!((ratio / start - 1.0).abs() < 0.05, "size drifted: {ratio} vs {start}");
        }
    }

    #[test]
    fn breakaway_drops_back_and_down() {
        let t = body_track("breakaway", level);
        let (first, last) = (t[0].0, t[t.len() - 1].0);
        assert!(last.z > first.z + 2.0 * 20.0, "didn't fall back: {first:?} -> {last:?}");
        assert!(last.y < first.y, "didn't descend: {first:?} -> {last:?}");
    }

    #[test]
    fn fin_cam_is_above_the_tail_looking_forward() {
        for (body, pose) in body_track("fin_cam", cruising) {
            assert!(body.x.abs() < 1e-6 && body.y > 0.22 * 20.0 && body.z > 0.3 * 20.0, "{body:?}");
            let s = cruising(0.0);
            let fwd = Attitude::new(pose.heading, pose.pitch, pose.roll).rotate(Vec3::FORWARD);
            assert!(fwd.dot(s.att.rotate(Vec3::FORWARD)) > 0.9, "not looking forward");
        }
    }

    #[test]
    fn planespotter_is_far_off_the_track_on_a_long_lens() {
        let t = body_track("planespotter", level);
        let first = t[0].1.pos;
        assert!(t.iter().all(|(_, p)| (p.pos - first).len() < 1e-6), "spotter moved");
        assert!(first.z.abs() > 6.0 * 1.5 * 20.0 - 1e-6, "too close to the track: {first:?}");
        assert!(t.iter().all(|(_, p)| p.zoom > 3.0 * Settings::default().zoom_scale - 1e-9));
    }

    fn aim_of(pose: &CameraPose) -> Vec3 {
        Attitude::new(pose.heading, pose.pitch, pose.roll).rotate(Vec3::FORWARD)
    }

    fn off_axis_deg(pose: &CameraPose, target: Vec3) -> f64 {
        aim_of(pose).dot((target - pose.pos).normalized_or(Vec3::UP)).clamp(-1.0, 1.0).acos().to_degrees()
    }

    #[test]
    fn front_orbit_crosses_ahead_of_the_nose() {
        let t = body_track("front_orbit", level);
        let (first, last) = (t[0].0, t[t.len() - 1].0);
        assert!(first.x * last.x < 0.0, "didn't cross from one side to the other: {first:?} -> {last:?}");
        assert!(first.z < 0.0 && last.z < 0.0, "not ahead of the wing");
        assert!(t.iter().any(|(b, _)| b.z < -1.5 * 1.5 * 20.0 && b.x.abs() < 5.0), "never right ahead of the nose");
    }

    #[test]
    fn belly_cam_is_under_the_nose_looking_ahead() {
        for (body, pose) in body_track("belly_cam", cruising) {
            assert!(body.y < 0.0 && body.z < 0.0 && body.x.abs() < 1e-6, "{body:?}");
            assert!(aim_of(&pose).dot(cruising(0.0).att.rotate(Vec3::FORWARD)) > 0.9, "not looking ahead");
        }
        // Taxiing, where this 20 m test aircraft's belly would be underground.
        for (_, pose) in body_track("belly_cam", taxiing) {
            assert!(pose.pos.y >= MOUNT_CLEARANCE_M - 1e-9, "underground: {}", pose.pos.y);
        }
    }

    #[test]
    fn shoulder_cam_is_above_looking_forward() {
        for (body, pose) in body_track("shoulder", cruising) {
            assert!(body.y > 0.13 * 20.0 && body.x.abs() > 1.0, "{body:?}");
            assert!(aim_of(&pose).dot(cruising(0.0).att.rotate(Vec3::FORWARD)) > 0.9, "not looking forward");
        }
    }

    /// On a 3° approach at 70 m/s, 60 m up.
    fn approaching(t: f64) -> Subject {
        let slope = 3.0_f64.to_radians();
        let vel = Vec3::new(70.0 * slope.cos(), -70.0 * slope.sin(), 0.0);
        let pos = Vec3::new(0.0, 60.0, 0.0) + vel * t;
        Subject { pos, vel, att: Attitude::new(90.0, -1.0, 0.0), agl: pos.y, on_ground: false, size: 20.0, systems: Systems::default() }
    }

    #[test]
    fn under_the_approach_stands_on_the_ground_below_the_track() {
        let t = body_track("under_approach", approaching);
        let cam = t[0].1.pos;
        assert!(t.iter().all(|(_, p)| (p.pos - cam).len() < 1e-6), "camera moved");
        // Eye height, raised to the usual ground clearance.
        assert!(cam.y >= EYE_HEIGHT_M && cam.y <= 2.0 + 1e-9, "not standing on the ground: {cam:?}");
        assert!(cam.x > 100.0 && cam.z.abs() < 0.5 * 1.5 * 20.0, "not under the track ahead: {cam:?}");
        // Only low: not at cruise altitude.
        let mut d = Director::new(only("under_approach"), 1);
        d.update(&cruising(0.0), DT, &mut flat);
        assert_ne!(d.current_shot().unwrap().id, "under_approach");
    }

    #[test]
    fn sun_silhouette_puts_the_low_sun_behind_the_aircraft() {
        let s = level(0.0);
        // No sun, or a high one: not filmed.
        for sun in [None, Some(40.0)] {
            let mut d = Director::new(only("sun_silhouette"), 1);
            if let Some(elevation) = sun {
                d.set_sun(200.0, elevation);
            }
            d.update(&s, DT, &mut flat);
            assert_ne!(d.current_shot().unwrap().id, "sun_silhouette", "sun {sun:?}");
        }
        let mut d = Director::new(only("sun_silhouette"), 1);
        d.set_sun(200.0, 5.0);
        let sun = d.low_sun.unwrap();
        for i in 0..200 {
            let s = level(i as f64 * DT);
            let pose = d.update(&s, DT, &mut flat);
            assert_eq!(d.current_shot().unwrap().id, "sun_silhouette");
            let to_aircraft = (s.pos - pose.pos).normalized_or(Vec3::UP);
            let angle = to_aircraft.dot(sun).clamp(-1.0, 1.0).acos().to_degrees();
            assert!(angle < 15.0, "sun {angle:.1}° off the aircraft");
        }
    }

    #[test]
    fn drone_reveal_tilts_up_onto_the_aircraft() {
        let t = body_track("drone_reveal", level);
        let at = |i: usize| off_axis_deg(&t[i].1, level(i as f64 * DT).pos);
        assert!(at(0) > 35.0, "aircraft in frame from the start: {:.1}°", at(0));
        assert!(at(t.len() - 1) < 5.0, "never found the aircraft: {:.1}°", at(t.len() - 1));
        assert!(t[0].1.pitch < -20.0, "doesn't start looking down at the scenery: {}", t[0].1.pitch);
    }

    #[test]
    fn locked_off_frame_never_moves_and_the_aircraft_crosses_it() {
        let t = body_track("locked_off", level);
        let p0 = t[0].1;
        for (_, p) in &t {
            assert!((p.pos - p0.pos).len() < 1e-9 && p.heading == p0.heading && p.pitch == p0.pitch, "camera moved");
        }
        let offs: Vec<f64> = (0..t.len()).map(|i| off_axis_deg(&t[i].1, level(i as f64 * DT).pos)).collect();
        assert!(offs[0] > 30.0 && offs[offs.len() - 1] > 30.0, "aircraft not outside the frame at both ends");
        assert!(offs.iter().any(|&o| o < 2.0), "aircraft never crosses the centre");
    }

    fn shot_of(d: &Director) -> &'static Shot {
        d.current_shot().unwrap()
    }

    fn with(systems: Systems, s: Subject) -> Subject {
        Subject { systems, ..s }
    }

    #[test]
    fn action_shots_are_never_picked_at_random() {
        for seed in 0..6 {
            let mut d = Director::new(Settings::default(), seed);
            d.set_engines(vec![Vec3::new(5.0, -1.0, -1.0), Vec3::new(-5.0, -1.0, -1.0)]);
            for subject in [cruising as fn(f64) -> Subject, taxiing] {
                for (_, _, id) in run(&mut d, 200.0, subject) {
                    assert!(CATALOG.iter().find(|s| s.id == id).unwrap().trigger.is_none(), "{id} picked at random");
                }
            }
        }
    }

    /// Cruising with the gear up until `before` s, then extending over 6 s.
    fn gear_down_at(before: f64) -> impl Fn(f64) -> Subject {
        move |t| {
            let gear = ((t - before) / 6.0).clamp(0.0, 1.0);
            with(Systems { gear, ..Systems::default() }, cruising(t))
        }
    }

    #[test]
    fn gear_extending_cuts_straight_to_a_gear_shot() {
        for seed in 0..6 {
            let mut d = Director::new(Settings::default(), seed);
            let s = gear_down_at(3.0);
            let mut t = 0.0;
            let mut cut_at = None;
            while t < 12.0 {
                d.update(&s(t), DT, &mut flat);
                if cut_at.is_none() && shot_of(&d).trigger == Some(Trigger::Gear) {
                    cut_at = Some(t);
                }
                if let Some(c) = cut_at {
                    if t - c < ACTION_MIN_TAKE_S - 2.0 * DT {
                        assert_eq!(shot_of(&d).trigger, Some(Trigger::Gear), "gear shot cut early at {t:.2}");
                    }
                }
                t += DT;
            }
            let c = cut_at.expect("no gear shot");
            assert!((3.0..3.5).contains(&c), "gear shot at {c:.2} s, gear moved at 3 s");
        }
    }

    #[test]
    fn gear_shots_film_the_wheels() {
        for seed in 0..6 {
            let mut d = Director::new(Settings { pause_s: 0.0, ..Settings::default() }, seed);
            let wheel = Vec3::new(3.0, -2.0, 0.5);
            d.set_gear(vec![Vec3::new(0.0, -2.0, -6.0), wheel, wheel.mirror_x(-1.0)]);
            let s = gear_down_at(1.0);
            let mut t = 0.0;
            while t < 6.0 {
                let now = s(t);
                let pose = d.update(&now, DT, &mut flat);
                let shot = shot_of(&d);
                if shot.trigger == Some(Trigger::Gear) && t > 2.5 {
                    // Aimed near a wheel, not the middle of the aircraft.
                    let near = [Vec3::new(0.0, -2.0, -6.0), wheel, wheel.mirror_x(-1.0)]
                        .iter()
                        .map(|w| off_axis_deg(&pose, now.pos + now.att.rotate(*w)))
                        .fold(f64::INFINITY, f64::min);
                    assert!(near < 25.0, "{}: wheel {near:.1}° off-axis", shot.id);
                }
                t += DT;
            }
        }
    }

    #[test]
    fn the_nose_gear_shot_needs_a_nose_wheel() {
        let mut d = Director::new(Settings::default(), 1);
        let nose = CATALOG.iter().position(|s| s.id == "gear_nose").unwrap();
        // A taildragger: two mains, one tailwheel.
        d.set_gear(vec![Vec3::new(1.2, -1.5, -0.5), Vec3::new(-1.2, -1.5, -0.5), Vec3::new(0.0, -0.6, 5.0)]);
        assert!(!d.supported(nose, 11.0));
        d.set_gear(vec![Vec3::new(1.2, -1.5, 0.5), Vec3::new(-1.2, -1.5, 0.5), Vec3::new(0.0, -1.5, -3.0)]);
        assert!(d.supported(nose, 11.0));
    }

    /// Wheel height above the runway `t` s into a landing: sinking 3.5 m/s
    /// from 40 m, flaring below 12 m to touch at about 1 m/s.
    fn landing_height(t: f64) -> (f64, f64) {
        let (mut h, mut time) = (40.0_f64, 0.0);
        let sink_at = |h: f64| if h > 12.0 { 3.5 } else { 1.0 + 2.5 * h / 12.0 };
        let step = 0.005;
        while time < t && h > 0.0 {
            h -= sink_at(h) * step;
            time += step;
        }
        if h <= 0.0 {
            (0.0, 0.0)
        } else {
            (h, sink_at(h))
        }
    }

    /// The landing at 70 m/s, with the reference point 2.4 m above the wheels.
    fn landing(t: f64) -> Subject {
        let rest = 2.4;
        let (h, sink) = landing_height(t);
        Subject {
            pos: Vec3::new(70.0 * t, rest + h, 0.0),
            vel: Vec3::new(70.0, -sink, 0.0),
            att: Attitude::new(90.0, 2.0, 0.0),
            agl: rest + h,
            on_ground: h <= 0.0,
            size: 20.0,
            systems: Systems::default(),
        }
    }

    fn touchdown_time() -> f64 {
        let mut t = 0.0;
        while !landing(t).on_ground {
            t += DT;
        }
        t
    }

    #[test]
    fn touchdown_is_filmed_from_before_the_wheels_touch_into_the_rollout() {
        let touch = touchdown_time();
        for seed in 0..8 {
            let mut d = Director::new(Settings::default(), seed);
            let mut cut_at = None;
            let mut t = 0.0;
            while t < touch + ROLLOUT_S - 0.5 {
                d.update(&landing(t), DT, &mut flat);
                let on = shot_of(&d).trigger == Some(Trigger::Touchdown);
                if on && cut_at.is_none() {
                    cut_at = Some(t);
                }
                if t >= touch - 1.0 && t <= touch + 1.0 {
                    assert!(on, "seed {seed}: {} on screen at touchdown {:+.1} s", shot_of(&d).id, t - touch);
                }
                t += DT;
            }
            let lead = touch - cut_at.unwrap();
            assert!((1.5..6.0).contains(&lead), "cut {lead:.1} s before touchdown");
        }
    }

    #[test]
    fn runway_edge_camera_stands_where_the_wheels_touch() {
        let touch = touchdown_time();
        let mut d = Director::new(only("touchdown_runway"), 4);
        let mut t = 0.0;
        let mut filmed = false;
        while t < touch + DT {
            let s = landing(t);
            let pose = d.update(&s, DT, &mut flat);
            if shot_of(&d).id == "touchdown_runway" && s.on_ground {
                let along = s.pos.x - pose.pos.x;
                assert!(along.abs() < 2.0 * 20.0, "aircraft {along:.0} m from abeam the camera at touchdown");
                assert!(pose.pos.y < 2.5, "camera not on the ground: {}", pose.pos.y);
                assert!(off_axis_deg(&pose, s.pos) < 15.0, "aircraft out of frame");
                filmed = true;
            }
            t += DT;
        }
        assert!(filmed, "runway edge shot never filmed the touchdown");
    }

    #[test]
    fn reversers_opening_cut_to_a_reverser_shot() {
        let mut d = Director::new(Settings::default(), 2);
        d.set_engines(vec![Vec3::new(5.0, -1.0, -1.0), Vec3::new(-5.0, -1.0, -1.0)]);
        let mut cut = None;
        for i in 0..150 {
            let t = i as f64 * DT;
            let reverser = ((t - 2.0) / 1.5).clamp(0.0, 1.0);
            d.update(&with(Systems { reverser, ..Systems::default() }, taxiing(t)), DT, &mut flat);
            if cut.is_none() && shot_of(&d).trigger == Some(Trigger::Reverser) {
                cut = Some(t);
            }
        }
        let c = cut.expect("no reverser shot");
        assert!((2.0..2.4).contains(&c), "reverser shot at {c:.2} s");
    }

    #[test]
    fn no_engine_data_no_reverser_shot() {
        let mut d = Director::new(Settings::default(), 2);
        for i in 0..150 {
            let t = i as f64 * DT;
            let reverser = ((t - 2.0) / 1.5).clamp(0.0, 1.0);
            d.update(&with(Systems { reverser, ..Systems::default() }, taxiing(t)), DT, &mut flat);
            assert!(shot_of(&d).trigger.is_none());
        }
    }

    #[test]
    fn an_action_waits_for_the_one_on_screen_then_plays() {
        let mut d = Director::new(Settings::default(), 9);
        let mut seen = Vec::new();
        for i in 0..(14.0 / DT) as usize {
            let t = i as f64 * DT;
            // Flaps at 2 s, gear 1 s later.
            let flaps = ((t - 2.0) / 4.0).clamp(0.0, 0.5);
            let gear = ((t - 3.0) / 6.0).clamp(0.0, 1.0);
            d.update(&with(Systems { gear, flaps, reverser: 0.0 }, cruising(t)), DT, &mut flat);
            let trig = shot_of(&d).trigger;
            if seen.last() != Some(&trig) {
                seen.push(trig);
            }
        }
        assert_eq!(seen[1..3], [Some(Trigger::Flaps), Some(Trigger::Gear)], "{seen:?}");
    }

    #[test]
    fn hold_and_unticked_action_shots_are_respected() {
        let s = gear_down_at(1.0);
        // Held: no cut.
        let mut d = Director::new(Settings::default(), 1);
        d.update(&s(0.0), DT, &mut flat);
        d.set_hold(true);
        let held = shot_of(&d).id;
        for i in 0..200 {
            d.update(&s(i as f64 * DT), DT, &mut flat);
            assert_eq!(shot_of(&d).id, held);
        }
        // All gear shots unticked: no gear shot.
        let mut settings = Settings::default();
        for id in ["gear_below", "gear_nose", "gear_cam"] {
            settings.set_disabled(id, true);
        }
        let mut d = Director::new(settings, 1);
        for i in 0..300 {
            d.update(&s(i as f64 * DT), DT, &mut flat);
            assert!(shot_of(&d).trigger.is_none());
        }
    }

    #[test]
    fn action_shots_can_be_previewed() {
        let mut d = Director::new(Settings::default(), 1);
        for id in ["gear_below", "flaps_window", "touchdown_chase"] {
            let i = CATALOG.iter().position(|s| s.id == id).unwrap();
            d.play(i);
            d.update(&cruising(0.0), DT, &mut flat);
            assert_eq!(shot_of(&d).id, id);
        }
    }
}
