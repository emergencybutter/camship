//! Spots the moments worth a dedicated shot: gear and flaps starting to
//! move, the wheels about to meet the runway, rotation, the reversers and
//! spoilers opening, an engine starting, a steep turn, a go-around...
//!
//! Touchdown is *predicted*, a few seconds ahead, so the camera is already
//! in place when the wheels touch; cutting on the touchdown itself would
//! miss it.

use crate::director::Subject;
use crate::math::{exp_alpha, Vec3};

/// Something happening to the aircraft that deserves its own shot.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Trigger {
    /// The landing gear starts to extend or retract.
    Gear,
    /// The flaps start to move.
    Flaps,
    /// The wheels are about to touch down.
    Touchdown,
    /// The thrust reversers open.
    Reverser,
    /// The nose comes up on the takeoff run (or a touch-and-go).
    Rotation,
    /// Accelerating down the runway, power up.
    TakeoffRoll,
    /// The ground spoilers pop up.
    Spoilers,
    /// An engine lights up.
    EngineStart,
    /// The last engine stops, parked.
    Shutdown,
    /// Banked past 45°.
    SteepTurn,
    /// The landing lights come on (only filmed in low light).
    Lights,
    /// A door or the canopy opens.
    Doors,
    /// Rolling backwards on the ground.
    Pushback,
    /// Climbing away from a low approach.
    GoAround,
}

impl Trigger {
    pub fn label(self) -> &'static str {
        match self {
            Trigger::Gear => "gear",
            Trigger::Flaps => "flaps",
            Trigger::Touchdown => "touchdown",
            Trigger::Reverser => "reversers",
            Trigger::Rotation => "rotation",
            Trigger::TakeoffRoll => "takeoff roll",
            Trigger::Spoilers => "ground spoilers",
            Trigger::EngineStart => "engine start",
            Trigger::Shutdown => "shutdown",
            Trigger::SteepTurn => "steep turn",
            Trigger::Lights => "landing lights",
            Trigger::Doors => "doors",
            Trigger::Pushback => "pushback",
            Trigger::GoAround => "go-around",
        }
    }

    /// When several actions compete, the higher one is filmed: the ones
    /// that are over in a second or two first.
    pub fn priority(self) -> u8 {
        match self {
            Trigger::Touchdown => 14,
            Trigger::Rotation => 13,
            Trigger::GoAround => 12,
            Trigger::Reverser => 11,
            Trigger::Spoilers => 10,
            Trigger::Gear => 9,
            Trigger::TakeoffRoll => 8,
            Trigger::Flaps => 7,
            Trigger::EngineStart => 6,
            Trigger::SteepTurn => 5,
            Trigger::Lights => 4,
            Trigger::Doors => 3,
            Trigger::Pushback => 2,
            Trigger::Shutdown => 1,
        }
    }
}

/// The state of the aircraft's moving parts and switches.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Systems {
    /// Landing gear, 0 = up, 1 = down (1 on fixed gear).
    pub gear: f64,
    /// Flaps, 0 = retracted, 1 = fully extended.
    pub flaps: f64,
    /// The most deployed thrust reverser, 0 = stowed.
    pub reverser: f64,
    /// Speedbrakes / spoilers, 0 = retracted, 1 = fully extended.
    pub speedbrake: f64,
    /// The brightest landing light, 0 = off, 1 = full.
    pub landing_lights: f64,
    /// The most open door or canopy, 0 = closed.
    pub doors: f64,
    /// Bit `i` set: engine `i` is burning fuel.
    pub engines_running: u16,
}

impl Default for Systems {
    fn default() -> Self {
        Self { gear: 1.0, flaps: 0.0, reverser: 0.0, speedbrake: 0.0, landing_lights: 0.0, doors: 0.0, engines_running: 0 }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Event {
    pub trigger: Trigger,
    /// Touchdown only: seconds until the wheels are expected to touch.
    pub in_s: f64,
    /// For the log, e.g. "extending".
    pub what: &'static str,
    /// Film from this side (+1 right, -1 left), e.g. the low wing in a turn.
    pub side: Option<f64>,
    /// The engine (slot index) this is about.
    pub engine: Option<usize>,
}

impl Event {
    fn new(trigger: Trigger, what: &'static str) -> Self {
        Self { trigger, in_s: 0.0, what, side: None, engine: None }
    }
}

/// A surface that moves from one position to another over a few seconds
/// (gear, flaps): reports once when it starts moving, then waits for it to
/// settle before it can report again.
#[derive(Clone, Copy, Debug, Default)]
struct Mover {
    settled: Option<f64>,
    moving: bool,
    last: f64,
    still_s: f64,
}

/// A move smaller than this is noise, not the surface moving.
const MOVE_THRESHOLD: f64 = 0.03;
/// Still for this long: the move is over.
const SETTLE_S: f64 = 0.8;
/// Slower than this (ratio per second) is still. Slow flaps take about
/// 30 s for the full travel, 0.03 per second.
const STILL_RATE: f64 = 0.005;

impl Mover {
    /// Returns +1 when it starts extending, -1 when it starts retracting.
    fn update(&mut self, v: f64, dt: f64) -> Option<f64> {
        if !v.is_finite() {
            return None;
        }
        let Some(settled) = self.settled else {
            self.settled = Some(v);
            self.last = v;
            return None;
        };
        let step = (v - self.last).abs();
        self.last = v;
        if self.moving {
            if dt > 0.0 && step < STILL_RATE * dt {
                self.still_s += dt;
                if self.still_s >= SETTLE_S {
                    self.moving = false;
                    self.settled = Some(v);
                }
            } else {
                self.still_s = 0.0;
            }
            None
        } else if (v - settled).abs() > MOVE_THRESHOLD {
            self.moving = true;
            self.still_s = 0.0;
            Some((v - settled).signum())
        } else {
            None
        }
    }
}

/// Something that switches on past `on` and must drop below `off` before
/// it can report again (reversers, spoilers, lights, doors).
#[derive(Clone, Copy, Debug, Default)]
struct Latch {
    armed: Option<bool>,
}

impl Latch {
    fn update(&mut self, v: f64, on: f64, off: f64) -> bool {
        if !v.is_finite() {
            return false;
        }
        // Already on when first seen: wait for it to go off first.
        let armed = self.armed.get_or_insert(v < off);
        if v < off {
            *armed = true;
            false
        } else if *armed && v > on {
            *armed = false;
            true
        } else {
            false
        }
    }
}

/// Fire the touchdown shot when the wheels are this many seconds from the
/// ground at the current sink rate...
const TOUCHDOWN_LEAD_S: f64 = 2.5;
/// ...but the flare slows the sink, so they really touch later than that.
const FLARE_STRETCH: f64 = 1.4;
/// Only this close to the ground (in sizes, or meters if more): a steep
/// dive at altitude is not a landing.
const TOUCHDOWN_MAX_HEIGHT: f64 = 1.0;
const TOUCHDOWN_MAX_HEIGHT_M: f64 = 20.0;
/// A touchdown is only predicted again after climbing back above this
/// (sizes, or meters if more), so a bounce isn't filmed twice.
const REARM_HEIGHT: f64 = 0.3;
const REARM_HEIGHT_M: f64 = 5.0;
/// Time constants smoothing the measured acceleration and pitch rate, s.
const ACCEL_TAU: f64 = 0.5;
const PITCH_RATE_TAU: f64 = 0.25;
/// Faster changes than these are jumps (a replay scrubbed, the aircraft
/// moved), not flying: m/s², °/s.
const MAX_ACCEL: f64 = 15.0;
const MAX_PITCH_RATE: f64 = 30.0;
/// Rotation: nose coming up this fast (°/s) at this speed (m/s), still
/// accelerating (m/s²), so a bounce on landing doesn't count.
const ROTATION_RATE: f64 = 1.5;
const ROTATION_MIN_SPEED: f64 = 20.0;
const ROTATION_MIN_ACCEL: f64 = 0.3;
/// Takeoff roll: past this speed while still accelerating this hard,
/// having been nearly stopped (below the other speed) since the last one.
const ROLL_MIN_SPEED: f64 = 12.0;
const ROLL_MIN_ACCEL: f64 = 1.2;
const ROLL_REARM_SPEED: f64 = 6.0;
/// Steep turn: banked past this for this long; again once back below the other.
const STEEP_BANK: f64 = 45.0;
const STEEP_S: f64 = 1.0;
const STEEP_REARM_BANK: f64 = 25.0;
/// Pushback: rolling backwards faster than this (m/s) for this long.
const PUSH_SPEED: f64 = 0.3;
const PUSH_S: f64 = 1.0;
/// Shutdown and doors: only when parked (m/s).
const PARKED_SPEED: f64 = 2.0;
/// Go-around: having been descending below the first height (sizes, or
/// meters if more) in the last `GA_MEMORY_S`, climbing faster than
/// `GA_CLIMB` m/s for `GA_CLIMB_S` below the second height.
const GA_LOW: f64 = 2.0;
const GA_LOW_M: f64 = 60.0;
const GA_HIGH: f64 = 5.0;
const GA_HIGH_M: f64 = 150.0;
const GA_MEMORY_S: f64 = 20.0;
const GA_CLIMB: f64 = 2.5;
const GA_CLIMB_S: f64 = 1.0;

#[derive(Clone, Debug, Default)]
pub struct Detector {
    gear: Mover,
    flaps: Mover,
    reverser: Latch,
    spoilers: Latch,
    lights: Latch,
    doors: Latch,
    touchdown_armed: Option<bool>,
    rotation_armed: bool,
    roll_armed: bool,
    steep_armed: bool,
    push_armed: bool,
    /// Height above ground of the reference point sitting on the wheels,
    /// meters, learned while on the ground.
    rest_agl: Option<f64>,
    /// Previous speed and pitch, for the rates below.
    prev: Option<(f64, f64)>,
    /// Smoothed rate of change of speed, m/s².
    accel: f64,
    /// Smoothed pitch rate, °/s.
    pitch_rate: f64,
    steep_s: f64,
    push_s: f64,
    engines: Option<u16>,
    /// Seconds since last seen descending low, for go-arounds.
    low_descent_s: Option<f64>,
    climb_s: f64,
}

impl Detector {
    /// Forgets everything: the next update only takes a baseline.
    pub fn reset(&mut self) {
        let rest = self.rest_agl;
        *self = Self { rest_agl: rest, ..Self::default() };
    }

    /// `rest_guess`: height above ground of the reference point when the
    /// wheels touch, meters, until it can be measured on the ground.
    /// Returns every action that started this frame.
    pub fn update(&mut self, s: &Subject, dt: f64, rest_guess: f64) -> Vec<Event> {
        let sys = s.systems;
        let mut events = Vec::new();
        if s.on_ground && s.agl.is_finite() {
            // Smoothed: the struts bounce.
            let a = s.agl.max(0.0);
            self.rest_agl = Some(self.rest_agl.map_or(a, |r| r + (a - r) * 0.1));
        }
        let rest = self.rest_agl.unwrap_or(rest_guess);
        let height = s.agl - rest;
        let speed = s.speed();
        let sink = -s.vel.y;
        let first = self.prev.is_none();
        if let Some((v0, p0)) = self.prev {
            if dt > 0.0 {
                let (accel, pitch_rate) = ((speed - v0) / dt, (s.att.pitch - p0) / dt);
                if accel.abs() <= MAX_ACCEL && pitch_rate.abs() <= MAX_PITCH_RATE {
                    self.accel += (accel - self.accel) * exp_alpha(dt, ACCEL_TAU);
                    self.pitch_rate += (pitch_rate - self.pitch_rate) * exp_alpha(dt, PITCH_RATE_TAU);
                }
            }
        }
        self.prev = Some((speed, s.att.pitch));

        // Touchdown, predicted.
        let armed = self.touchdown_armed.get_or_insert(!s.on_ground);
        if s.on_ground {
            *armed = false;
        } else if height > (REARM_HEIGHT * s.size).max(REARM_HEIGHT_M) {
            *armed = true;
        }
        if *armed
            && !s.on_ground
            && sink > 0.3
            && height <= (TOUCHDOWN_MAX_HEIGHT * s.size).max(TOUCHDOWN_MAX_HEIGHT_M)
            && height.max(0.0) / sink <= TOUCHDOWN_LEAD_S
        {
            *armed = false;
            let in_s = height.max(0.0) / sink * FLARE_STRETCH;
            events.push(Event { in_s, ..Event::new(Trigger::Touchdown, "about to touch down") });
        }

        // Rotation: armed by being in the air or slow, so it fires once per
        // takeoff and again on a touch-and-go.
        if !s.on_ground || speed < ROLL_REARM_SPEED {
            self.rotation_armed = true;
        }
        if s.on_ground
            && self.rotation_armed
            && !first
            && speed > ROTATION_MIN_SPEED
            && self.accel > ROTATION_MIN_ACCEL
            && self.pitch_rate > ROTATION_RATE
        {
            self.rotation_armed = false;
            events.push(Event::new(Trigger::Rotation, "nose coming up"));
        }

        // Takeoff roll.
        if s.on_ground && speed < ROLL_REARM_SPEED {
            self.roll_armed = true;
        }
        if s.on_ground && self.roll_armed && speed > ROLL_MIN_SPEED && self.accel > ROLL_MIN_ACCEL {
            self.roll_armed = false;
            events.push(Event::new(Trigger::TakeoffRoll, "accelerating"));
        }

        // Go-around.
        let low = (GA_LOW * s.size).max(GA_LOW_M);
        if s.on_ground {
            self.low_descent_s = None;
        } else if height < low && sink > 1.5 {
            self.low_descent_s = Some(0.0);
        } else if let Some(t) = self.low_descent_s.as_mut() {
            *t += dt;
            if *t > GA_MEMORY_S {
                self.low_descent_s = None;
            }
        }
        if !s.on_ground && sink < -GA_CLIMB && height < (GA_HIGH * s.size).max(GA_HIGH_M) {
            self.climb_s += dt;
        } else {
            self.climb_s = 0.0;
        }
        if self.low_descent_s.is_some() && self.climb_s >= GA_CLIMB_S {
            self.low_descent_s = None;
            events.push(Event::new(Trigger::GoAround, "climbing away"));
        }

        if let Some(dir) = self.gear.update(sys.gear, dt) {
            let what = if dir > 0.0 { "extending" } else { "retracting" };
            events.push(Event::new(Trigger::Gear, what));
        }
        if let Some(dir) = self.flaps.update(sys.flaps, dt) {
            let what = if dir > 0.0 { "extending" } else { "retracting" };
            events.push(Event::new(Trigger::Flaps, what));
        }
        if self.reverser.update(sys.reverser, 0.1, 0.02) {
            events.push(Event::new(Trigger::Reverser, "opening"));
        }
        // Ground spoilers; in flight they're speedbrakes, not worth a cut.
        if self.spoilers.update(if s.on_ground { sys.speedbrake } else { 0.0 }, 0.5, 0.2) {
            events.push(Event::new(Trigger::Spoilers, "deployed"));
        }
        if self.lights.update(sys.landing_lights, 0.5, 0.1) {
            events.push(Event::new(Trigger::Lights, "on"));
        }
        if self.doors.update(if speed < PARKED_SPEED { sys.doors } else { 0.0 }, 0.1, 0.02) {
            events.push(Event::new(Trigger::Doors, "opening"));
        }

        // Engines.
        let was = *self.engines.get_or_insert(sys.engines_running);
        let now = sys.engines_running;
        let started = now & !was;
        if started != 0 {
            let engine = started.trailing_zeros() as usize;
            events.push(Event { engine: Some(engine), ..Event::new(Trigger::EngineStart, "lighting up") });
        }
        if was != 0 && now == 0 && s.on_ground && speed < PARKED_SPEED {
            events.push(Event::new(Trigger::Shutdown, "all engines off"));
        }
        self.engines = Some(now);

        // Steep turn, filmed from the low wing's side.
        let bank = s.att.roll;
        if bank.abs() < STEEP_REARM_BANK {
            self.steep_armed = true;
        }
        if !s.on_ground && bank.abs() > STEEP_BANK {
            self.steep_s += dt;
        } else {
            self.steep_s = 0.0;
        }
        if self.steep_armed && self.steep_s >= STEEP_S {
            self.steep_armed = false;
            events.push(Event { side: Some(bank.signum()), ..Event::new(Trigger::SteepTurn, "banking hard") });
        }

        // Pushback: moving backwards along the nose.
        let along = s.vel.dot(s.att.rotate(Vec3::FORWARD));
        if !s.on_ground || speed < 0.1 || along > PUSH_SPEED {
            self.push_armed = true;
        }
        if s.on_ground && along < -PUSH_SPEED {
            self.push_s += dt;
        } else {
            self.push_s = 0.0;
        }
        if self.push_armed && self.push_s >= PUSH_S {
            self.push_armed = false;
            events.push(Event::new(Trigger::Pushback, "rolling back"));
        }

        events.sort_by_key(|e| std::cmp::Reverse(e.trigger.priority()));
        events
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Attitude;

    const DT: f64 = 1.0 / 30.0;

    fn subject(agl: f64, sink: f64, on_ground: bool, systems: Systems) -> Subject {
        Subject {
            pos: Vec3::new(0.0, agl, 0.0),
            vel: Vec3::new(70.0, -sink, 0.0),
            att: Attitude::new(90.0, 0.0, 0.0),
            agl,
            on_ground,
            size: 30.0,
            systems,
        }
    }

    fn parked(systems: Systems) -> Subject {
        Subject { vel: Vec3::ZERO, ..subject(3.0, 0.0, true, systems) }
    }

    /// The triggers fired over a run of subjects.
    fn fired(d: &mut Detector, run: impl IntoIterator<Item = Subject>) -> Vec<Trigger> {
        run.into_iter().flat_map(|s| d.update(&s, DT, 3.0)).map(|e| e.trigger).collect()
    }

    #[test]
    fn gear_reports_once_per_move_both_ways() {
        let mut d = Detector::default();
        let mut out = Vec::new();
        // Up, then down over 5 s, settle, then up again.
        let mut ratios = vec![0.0; 30];
        ratios.extend((0..150).map(|i| i as f64 / 150.0));
        ratios.extend(vec![1.0; 60]);
        ratios.extend((0..150).map(|i| 1.0 - i as f64 / 150.0));
        for g in ratios {
            let s = subject(500.0, 0.0, false, Systems { gear: g, ..Systems::default() });
            for e in d.update(&s, DT, 3.0) {
                out.push((e.trigger, e.what));
            }
        }
        assert_eq!(out, vec![(Trigger::Gear, "extending"), (Trigger::Gear, "retracting")]);
    }

    #[test]
    fn no_event_from_the_first_sample() {
        let mut d = Detector::default();
        let sys = Systems {
            flaps: 0.5,
            reverser: 1.0,
            speedbrake: 1.0,
            landing_lights: 1.0,
            doors: 1.0,
            engines_running: 3,
            ..Systems::default()
        };
        assert_eq!(fired(&mut d, [parked(sys), parked(sys)]), []);
    }

    #[test]
    fn flaps_stepping_in_stages_report_each_stage() {
        let mut d = Detector::default();
        let mut f: f64 = 0.0;
        let mut run = Vec::new();
        for stage in [0.25, 0.5] {
            for _ in 0..120 {
                f = (f + 0.005).min(stage);
                run.push(subject(500.0, 0.0, false, Systems { flaps: f, ..Systems::default() }));
            }
        }
        assert_eq!(fired(&mut d, run), [Trigger::Flaps, Trigger::Flaps]);
    }

    #[test]
    fn slow_flaps_report_once_per_move() {
        let mut d = Detector::default();
        // 0 to 1 over 30 s.
        let run = (0..(32.0 / DT) as usize).map(|i| {
            let flaps = (i as f64 * DT / 30.0).min(1.0);
            subject(500.0, 0.0, false, Systems { flaps, ..Systems::default() })
        });
        assert_eq!(fired(&mut d, run), [Trigger::Flaps]);
    }

    #[test]
    fn touchdown_is_predicted_seconds_ahead_and_only_once() {
        let mut d = Detector::default();
        let (mut agl, sink, rest) = (60.0, 3.5, 3.0);
        let mut at = Vec::new();
        let mut t = 0.0;
        while agl > rest {
            for e in d.update(&subject(agl, sink, false, Systems::default()), DT, rest) {
                assert_eq!(e.trigger, Trigger::Touchdown);
                at.push((t, e.in_s));
            }
            agl -= sink * DT;
            t += DT;
        }
        // Rolling out, slowing down: no more.
        for i in 0..100 {
            let s = Subject { vel: Vec3::new(70.0 - i as f64 * 0.1, 0.0, 0.0), ..subject(rest, 0.0, true, Systems::default()) };
            assert_eq!(d.update(&s, DT, rest), []);
        }
        assert_eq!(at.len(), 1, "{at:?}");
        let (when, in_s) = at[0];
        let real = t - when;
        assert!((2.0..=3.0).contains(&real), "fired {real:.1} s before touching");
        assert!(in_s >= real, "expected in {in_s:.1} s, touched in {real:.1} s");
    }

    #[test]
    fn a_dive_at_altitude_is_not_a_landing() {
        let mut d = Detector::default();
        let run = (0..300).map(|i| subject(3000.0 - 40.0 * i as f64 * DT, 40.0, false, Systems::default()));
        assert_eq!(fired(&mut d, run), []);
    }

    #[test]
    fn reversers_report_when_they_open() {
        let mut d = Detector::default();
        let run = [0.0, 0.0, 0.05, 0.2, 0.6, 1.0, 1.0, 0.5, 0.0, 0.0, 0.3, 1.0]
            .map(|r| subject(3.0, 0.0, true, Systems { reverser: r, ..Systems::default() }));
        assert_eq!(fired(&mut d, run), [Trigger::Reverser, Trigger::Reverser]);
    }

    /// A takeoff: lined up, accelerating at 2 m/s², rotating at 70 m/s at
    /// 3°/s to 8°, lifting off at 8°.
    fn takeoff(t: f64) -> Subject {
        let speed = (2.0 * t).min(80.0);
        let rotate_at = 35.0;
        let pitch = ((t - rotate_at) * 3.0).clamp(0.0, 8.0);
        let airborne = t > rotate_at + 3.5;
        let climb = if airborne { 4.0 } else { 0.0 };
        let agl = 3.0 + if airborne { (t - rotate_at - 3.5) * climb } else { 0.0 };
        Subject {
            pos: Vec3::new(0.0, agl, 0.0),
            vel: Vec3::new(speed, climb, 0.0),
            att: Attitude::new(90.0, pitch, 0.0),
            agl,
            on_ground: !airborne,
            size: 30.0,
            systems: Systems::default(),
        }
    }

    #[test]
    fn a_takeoff_is_a_roll_then_a_rotation() {
        let mut d = Detector::default();
        let run = (0..(45.0 / DT) as usize).map(|i| takeoff(i as f64 * DT));
        assert_eq!(fired(&mut d, run), [Trigger::TakeoffRoll, Trigger::Rotation]);
    }

    #[test]
    fn a_jump_in_speed_is_not_a_takeoff() {
        let mut d = Detector::default();
        let mut run = vec![parked(Systems::default()); 10];
        run.extend((0..60).map(|_| subject(3.0, 0.0, true, Systems::default())));
        assert_eq!(fired(&mut d, run), []);
    }

    #[test]
    fn taxiing_and_stopping_is_not_a_takeoff() {
        let mut d = Detector::default();
        // Speed up to 10 m/s and back down, nose bobbing on the brakes.
        let run = (0..600).map(|i| {
            let t = i as f64 * DT;
            let speed = 10.0 * (t / 20.0 * std::f64::consts::PI).sin().abs();
            Subject {
                vel: Vec3::new(speed, 0.0, 0.0),
                att: Attitude::new(90.0, (t * 3.0).sin(), 0.0),
                ..subject(3.0, 0.0, true, Systems::default())
            }
        });
        assert_eq!(fired(&mut d, run), []);
    }

    #[test]
    fn engines_starting_one_by_one_then_all_shutting_down() {
        let mut d = Detector::default();
        let mut events = Vec::new();
        for running in [0, 0, 2, 2, 3, 3, 1, 0, 0] {
            events.extend(d.update(&parked(Systems { engines_running: running, ..Systems::default() }), DT, 3.0));
        }
        let got: Vec<_> = events.iter().map(|e| (e.trigger, e.engine)).collect();
        assert_eq!(
            got,
            [(Trigger::EngineStart, Some(1)), (Trigger::EngineStart, Some(0)), (Trigger::Shutdown, None)]
        );
    }

    #[test]
    fn steep_turn_reports_the_low_wing() {
        let mut d = Detector::default();
        let mut sides = Vec::new();
        for bank in [0.0, -50.0, 0.0, 30.0, 55.0] {
            for _ in 0..45 {
                let s = Subject { att: Attitude::new(90.0, 0.0, bank), ..subject(1000.0, 0.0, false, Systems::default()) };
                sides.extend(d.update(&s, DT, 3.0).into_iter().map(|e| (e.trigger, e.side)));
            }
        }
        assert_eq!(sides, [(Trigger::SteepTurn, Some(-1.0)), (Trigger::SteepTurn, Some(1.0))]);
    }

    #[test]
    fn pushback_lights_spoilers_and_doors() {
        let mut d = Detector::default();
        let mut run = vec![parked(Systems::default()); 3];
        // Door opens, parked.
        run.push(parked(Systems { doors: 0.5, ..Systems::default() }));
        run.push(parked(Systems::default()));
        // Pushed back (nose east, moving west) for 2 s.
        run.extend((0..60).map(|_| Subject { vel: Vec3::new(-1.0, 0.0, 0.0), ..parked(Systems::default()) }));
        // Lights on.
        run.push(parked(Systems { landing_lights: 1.0, ..Systems::default() }));
        // Spoilers in flight: nothing; on the ground: yes.
        let brakes = Systems { landing_lights: 1.0, speedbrake: 1.0, ..Systems::default() };
        run.push(Subject { on_ground: false, agl: 500.0, ..parked(brakes) });
        run.push(parked(brakes));
        assert_eq!(fired(&mut d, run), [Trigger::Doors, Trigger::Pushback, Trigger::Lights, Trigger::Spoilers]);
    }

    #[test]
    fn climbing_away_from_a_low_approach_is_a_go_around() {
        let mut d = Detector::default();
        // Descending to 50 m, then climbing at 5 m/s.
        let mut run: Vec<Subject> = (0..300).map(|i| subject(80.0 - 3.0 * i as f64 * DT, 3.0, false, Systems::default())).collect();
        run.extend((0..90).map(|i| subject(50.0 + 5.0 * i as f64 * DT, -5.0, false, Systems::default())));
        assert_eq!(fired(&mut d, run), [Trigger::GoAround]);
        // A normal climb after takeoff isn't one.
        let mut d = Detector::default();
        let run = (0..300).map(|i| subject(10.0 + 5.0 * i as f64 * DT, -5.0, false, Systems::default()));
        assert_eq!(fired(&mut d, run), []);
    }
}
