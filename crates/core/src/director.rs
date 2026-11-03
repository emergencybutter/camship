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

use crate::math::{ease_in_out, exp_alpha, lerp, look_angles, Attitude, Rng, SmoothAttitude, Spring3, Vec3};
use crate::settings::Settings;
use crate::shots::{nacelle_size, Path, Shot, Target, CATALOG};

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
    /// Body-frame position (meters) of the engine an engine shot films.
    engine: Option<Vec3>,
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
    /// Unit vector towards the sun, world frame, when it's worth framing.
    sun: Option<Vec3>,
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
            sun: None,
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
        self.shots.get(index).is_some_and(|shot| shot.target == Target::Airframe || !self.engines_for(shot, size).is_empty())
    }

    /// Engine positions (body frame, meters; +x right, +y up, +z aft).
    /// Engine shots are only picked when this is not empty.
    pub fn set_engines(&mut self, engines: Vec<Vec3>) {
        self.engines = engines;
    }

    /// Where the sun is: true heading and elevation, degrees. Two-sided shots
    /// then favour the side that puts the sun behind the aircraft, so sunrises
    /// and sunsets end up in the frame.
    pub fn set_sun(&mut self, heading: f64, elevation: f64) {
        self.sun = (heading.is_finite()
            && elevation.is_finite()
            && (SUN_MIN_ELEVATION..=SUN_MAX_ELEVATION).contains(&elevation))
        .then(|| {
            let (h, p) = (heading.to_radians(), elevation.to_radians());
            Vec3::new(h.sin() * p.cos(), p.sin(), -h.cos() * p.cos())
        });
    }

    fn engines_for(&self, shot: &Shot, size: f64) -> Vec<Vec3> {
        match shot.target {
            Target::Airframe => Vec::new(),
            Target::Engine { outboard_only } => {
                self.engines.iter().copied().filter(|e| !outboard_only || e.x.abs() > 0.1 * size).collect()
            }
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

        let mut cut = false;
        let expired = match self.take {
            None => true,
            Some(t) => {
                self.skip
                    || (!self.hold && self.clock - t.start >= t.duration)
                    || !self.still_fits(&t, s)
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
            self.take = Some(self.new_take(shot, s, frame, ground));
            cut = true;
        }
        let take = self.take.expect("a take was just set");

        match self.shots[take.shot].path {
            // Rigid: no lag, no smoothing, no clearance push. It's bolted on.
            Path::Mount { at, offset_m } => return self.mounted(&take, s, at * s.size + offset_m),
            Path::Overhead { .. } => return self.overhead(&take, s, frame),
            _ => {}
        }

        let mut f = self.framing(&take, s, frame);
        f.pos = self.keep_clear(f.pos, s, ground);

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

    fn is_fixed(&self, take: &Take) -> bool {
        matches!(self.shots[take.shot].path, Path::Flyby { .. })
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
            zoom: (lerp(shot.zoom.0, shot.zoom.1, ease_in_out(self.progress(take))) * self.settings.zoom_scale).max(0.1),
        }
    }

    /// `at`: mount point in the body frame, meters.
    fn mounted(&mut self, take: &Take, s: &Subject, at: Vec3) -> CameraPose {
        let shot = &self.shots[take.shot];
        let e = ease_in_out(self.progress(take));
        let m = take.mirror;
        let mount = at.mirror_x(m);
        let look = (shot.look_from.lerp(shot.look_to, e) * s.size).mirror_x(m);
        // Aim in the body frame, then carry it through the aircraft's attitude.
        let (heading, pitch) = look_angles(look - mount);
        let local = Attitude::new(heading, pitch, shot.dutch * m);
        let world = s.att.compose(local);
        // The next shot starts with a cut, so it re-aims from scratch.
        self.aim = Spring3::default();
        self.sight = None;
        CameraPose {
            pos: s.pos + s.att.rotate(mount),
            heading: world.heading.rem_euclid(360.0),
            pitch: world.pitch.clamp(-89.9, 89.9),
            roll: world.roll,
            zoom: (lerp(shot.zoom.0, shot.zoom.1, e) * self.settings.zoom_scale).max(0.1),
        }
    }

    fn fits(&self, shot: &Shot, s: &Subject) -> bool {
        (if s.on_ground { shot.on_ground } else { shot.in_air })
            && s.agl >= shot.min_agl * s.size
            && s.speed() >= shot.min_speed
            && (shot.target == Target::Airframe || !self.engines_for(shot, s.size).is_empty())
    }

    fn still_fits(&self, take: &Take, s: &Subject) -> bool {
        // A fixed camera stays put whatever the aircraft does, until the take ends.
        self.is_fixed(take) || self.fits(&self.shots[take.shot], s)
    }

    fn any_enabled_fit(&self, s: &Subject) -> bool {
        self.shots.iter().any(|sh| !self.settings.is_disabled(sh.id) && self.fits(sh, s))
    }

    fn pick(&mut self, s: &Subject) -> usize {
        if let Some(i) = self.requested.take() {
            if self.fits(&self.shots[i], s) {
                return i;
            }
        }
        let current = self.take.map(|t| t.shot);
        let fitting: Vec<usize> = (0..self.shots.len()).filter(|&i| self.fits(&self.shots[i], s)).collect();
        let enabled: Vec<usize> =
            fitting.iter().copied().filter(|&i| !self.settings.is_disabled(self.shots[i].id)).collect();
        let fresh: Vec<usize> = enabled.iter().copied().filter(|i| !self.recent.contains(i)).collect();
        let not_current: Vec<usize> = enabled.iter().copied().filter(|&i| Some(i) != current).collect();
        let pool = [fresh, not_current, enabled, fitting].into_iter().find(|p| !p.is_empty());
        let Some(pool) = pool else {
            // Nothing fits (e.g. a size/agl edge case): fall back to a shot that works anywhere.
            return self.shots.iter().position(|sh| sh.on_ground && sh.in_air && sh.min_agl == 0.0).unwrap_or(0);
        };

        let total: f64 = pool.iter().map(|&i| self.shots[i].weight.max(0.0)).sum();
        let mut r = self.rng.next_f64() * total;
        let mut chosen = pool[pool.len() - 1];
        for &i in &pool {
            r -= self.shots[i].weight.max(0.0);
            if r < 0.0 {
                chosen = i;
                break;
            }
        }
        self.recent.push_back(chosen);
        while self.recent.len() > RECENT {
            self.recent.pop_front();
        }
        chosen
    }

    fn new_take(
        &mut self,
        shot: usize,
        s: &Subject,
        frame: Attitude,
        ground: &mut dyn FnMut(Vec3) -> Option<f64>,
    ) -> Take {
        let (lo, hi) = (self.settings.shot_min_s, self.settings.shot_max_s.max(self.settings.shot_min_s));
        let duration = self.rng.range(lo, hi);
        let spec = &self.shots[shot];
        let mut two_sided = spec.mirror;
        let candidates = self.engines_for(spec, s.size);
        let mut side = None;
        let engine = (!candidates.is_empty()).then(|| {
            let e = candidates[(self.rng.next_u64() % candidates.len() as u64) as usize];
            // Film each engine from its outboard side; centreline engines from either.
            if e.x.abs() > 0.05 * s.size {
                side = Some(e.x.signum());
                two_sided = false;
            }
            e
        });
        let take = |mirror: f64, ground: &mut dyn FnMut(Vec3) -> Option<f64>| Take {
            shot,
            start: self.clock,
            duration,
            mirror,
            anchor: self.anchor(shot, s, duration, mirror, ground),
            engine,
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

    /// World position of a fixed camera, for shots that have one.
    fn anchor(
        &self,
        shot: usize,
        s: &Subject,
        duration: f64,
        mirror: f64,
        ground: &mut dyn FnMut(Vec3) -> Option<f64>,
    ) -> Option<Vec3> {
        match self.shots[shot].path {
            Path::Flyby { side, height } => {
                let speed = s.speed();
                let fwd = if speed > 1.0 { s.vel * (1.0 / speed) } else { s.att.rotate(Vec3::FORWARD) };
                let flat = fwd.horizontal().normalized_or(Vec3::FORWARD);
                let right = Vec3::new(-flat.z, 0.0, flat.x);
                // The aircraft passes the photographer a little after mid-take.
                let lead = speed * duration * 0.55;
                let mut p = s.pos + fwd * lead + right * (side * s.size * self.settings.distance_scale * mirror);
                if s.on_ground {
                    let terrain = ground(p).unwrap_or(s.pos.y - s.agl);
                    p.y = terrain + EYE_HEIGHT_M;
                } else {
                    p.y += height * s.size;
                }
                Some(p)
            }
            _ => None,
        }
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
        // Engine shots are built around the engine, in nacelle sizes.
        let (origin, look_unit, reach) = match take.engine {
            Some(engine) => {
                let n = nacelle_size(size);
                (engine, n, n * self.settings.distance_scale)
            }
            None => (Vec3::ZERO, size, reach),
        };

        let pos = match shot.path {
            Path::Line { from, to } => s.pos + formation.rotate(origin + from.lerp(to, e).mirror_x(m) * reach),
            Path::Orbit { radius, az_from, az_to, height_from, height_to } => {
                let az = (lerp(az_from, az_to, e) * m).to_radians();
                let h = lerp(height_from, height_to, e);
                let off = Vec3::new(radius * az.sin(), h, -radius * az.cos()) * reach;
                s.pos + frame.level().rotate(off)
            }
            Path::Flyby { .. } => take.anchor.unwrap_or(s.pos),
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
                assert!(pose.pos.y >= 2.0 - 1e-9, "{id} below clearance: {}", pose.pos.y);
                // A mounted camera sits on the airframe by design.
                let mounted = matches!(CATALOG.iter().find(|sh| sh.id == id).unwrap().path, Path::Mount { .. });
                assert!(mounted || (pose.pos - s.pos).len() >= 0.5 * s.size - 1e-6, "{id} inside the aircraft");
            }
        }
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
            // Detail shots aim at a part of the airframe, so allow some margin.
            assert!(off_axis < 25.0, "{id}: aircraft {off_axis:.1}° off-axis");
        }
    }
}
