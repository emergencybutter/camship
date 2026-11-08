//! Spots the moments worth a dedicated shot: gear and flaps starting to
//! move, the wheels about to meet the runway, the reversers opening.
//!
//! Touchdown is *predicted*, a few seconds ahead, so the camera is already
//! in place when the wheels touch; cutting on the touchdown itself would
//! miss it.

use crate::director::Subject;

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
}

impl Trigger {
    pub fn label(self) -> &'static str {
        match self {
            Trigger::Gear => "gear",
            Trigger::Flaps => "flaps",
            Trigger::Touchdown => "touchdown",
            Trigger::Reverser => "reversers",
        }
    }
}

/// The state of the aircraft's moving parts, each 0..1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Systems {
    /// Landing gear, 0 = up, 1 = down (1 on fixed gear).
    pub gear: f64,
    /// Flaps, 0 = retracted, 1 = fully extended.
    pub flaps: f64,
    /// The most deployed thrust reverser, 0 = stowed.
    pub reverser: f64,
}

impl Default for Systems {
    fn default() -> Self {
        Self { gear: 1.0, flaps: 0.0, reverser: 0.0 }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Event {
    pub trigger: Trigger,
    /// Touchdown only: seconds until the wheels are expected to touch.
    pub in_s: f64,
    /// For the log, e.g. "extending".
    pub what: &'static str,
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
/// Reversers count as open past this, and closed again below the other.
const REVERSER_OPEN: f64 = 0.1;
const REVERSER_CLOSED: f64 = 0.02;

#[derive(Clone, Debug, Default)]
pub struct Detector {
    gear: Mover,
    flaps: Mover,
    reverser_armed: bool,
    touchdown_armed: Option<bool>,
    /// Height above ground of the reference point sitting on the wheels,
    /// meters, learned while on the ground.
    rest_agl: Option<f64>,
}

impl Detector {
    /// Forgets everything: the next update only takes a baseline.
    pub fn reset(&mut self) {
        let rest = self.rest_agl;
        *self = Self { rest_agl: rest, ..Self::default() };
    }

    /// `rest_guess`: height above ground of the reference point when the
    /// wheels touch, meters, until it can be measured on the ground.
    pub fn update(&mut self, s: &Subject, dt: f64, rest_guess: f64) -> Option<Event> {
        let sys = s.systems;
        if s.on_ground && s.agl.is_finite() {
            // Smoothed: the struts bounce.
            let a = s.agl.max(0.0);
            self.rest_agl = Some(self.rest_agl.map_or(a, |r| r + (a - r) * 0.1));
        }
        let rest = self.rest_agl.unwrap_or(rest_guess);
        let height = s.agl - rest;

        let mut events = Vec::with_capacity(1);

        // Touchdown, predicted.
        let armed = self.touchdown_armed.get_or_insert(!s.on_ground);
        if s.on_ground {
            *armed = false;
        } else if height > (REARM_HEIGHT * s.size).max(REARM_HEIGHT_M) {
            *armed = true;
        }
        let sink = -s.vel.y;
        if *armed
            && !s.on_ground
            && sink > 0.3
            && height <= (TOUCHDOWN_MAX_HEIGHT * s.size).max(TOUCHDOWN_MAX_HEIGHT_M)
            && height.max(0.0) / sink <= TOUCHDOWN_LEAD_S
        {
            *armed = false;
            let in_s = height.max(0.0) / sink * FLARE_STRETCH;
            events.push(Event { trigger: Trigger::Touchdown, in_s, what: "about to touch down" });
        }

        if let Some(dir) = self.gear.update(sys.gear, dt) {
            let what = if dir > 0.0 { "extending" } else { "retracting" };
            events.push(Event { trigger: Trigger::Gear, in_s: 0.0, what });
        }
        if let Some(dir) = self.flaps.update(sys.flaps, dt) {
            let what = if dir > 0.0 { "extending" } else { "retracting" };
            events.push(Event { trigger: Trigger::Flaps, in_s: 0.0, what });
        }

        if sys.reverser < REVERSER_CLOSED {
            self.reverser_armed = true;
        } else if self.reverser_armed && sys.reverser > REVERSER_OPEN {
            self.reverser_armed = false;
            events.push(Event { trigger: Trigger::Reverser, in_s: 0.0, what: "opening" });
        }

        // Touchdown first: it can't wait.
        events.into_iter().next()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Attitude, Vec3};

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

    #[test]
    fn gear_reports_once_per_move_both_ways() {
        let mut d = Detector::default();
        let mut fired = Vec::new();
        // Up, then down over 5 s, settle, then up again.
        let mut ratios = vec![0.0; 30];
        ratios.extend((0..150).map(|i| i as f64 / 150.0));
        ratios.extend(vec![1.0; 60]);
        ratios.extend((0..150).map(|i| 1.0 - i as f64 / 150.0));
        for g in ratios {
            let s = subject(500.0, 0.0, false, Systems { gear: g, ..Systems::default() });
            if let Some(e) = d.update(&s, DT, 3.0) {
                fired.push((e.trigger, e.what));
            }
        }
        assert_eq!(fired, vec![(Trigger::Gear, "extending"), (Trigger::Gear, "retracting")]);
    }

    #[test]
    fn no_event_from_the_first_sample() {
        let mut d = Detector::default();
        let s = subject(0.0, 0.0, true, Systems { gear: 1.0, flaps: 0.5, reverser: 1.0 });
        assert_eq!(d.update(&s, DT, 3.0), None);
    }

    #[test]
    fn flaps_stepping_in_stages_report_each_stage() {
        let mut d = Detector::default();
        let mut n = 0;
        let mut f: f64 = 0.0;
        for stage in [0.25, 0.5] {
            for _ in 0..120 {
                f = (f + 0.005).min(stage);
                let s = subject(500.0, 0.0, false, Systems { flaps: f, ..Systems::default() });
                n += d.update(&s, DT, 3.0).is_some() as usize;
            }
        }
        assert_eq!(n, 2);
    }

    #[test]
    fn slow_flaps_report_once_per_move() {
        let mut d = Detector::default();
        let mut n = 0;
        // 0 to 1 over 30 s.
        for i in 0..(32.0 / DT) as usize {
            let flaps = (i as f64 * DT / 30.0).min(1.0);
            n += d.update(&subject(500.0, 0.0, false, Systems { flaps, ..Systems::default() }), DT, 3.0).is_some() as usize;
        }
        assert_eq!(n, 1);
    }

    #[test]
    fn touchdown_is_predicted_seconds_ahead_and_only_once() {
        let mut d = Detector::default();
        let (mut agl, sink, rest) = (60.0, 3.5, 3.0);
        let mut at = Vec::new();
        let mut t = 0.0;
        while agl > rest {
            if let Some(e) = d.update(&subject(agl, sink, false, Systems::default()), DT, rest) {
                assert_eq!(e.trigger, Trigger::Touchdown);
                at.push((t, e.in_s));
            }
            agl -= sink * DT;
            t += DT;
        }
        // Rolling out: no more.
        for _ in 0..100 {
            assert_eq!(d.update(&subject(rest, 0.0, true, Systems::default()), DT, rest), None);
        }
        assert_eq!(at.len(), 1, "{at:?}");
        let (fired, in_s) = at[0];
        let real = t - fired;
        assert!((2.0..=3.0).contains(&real), "fired {real:.1} s before touching");
        assert!(in_s >= real, "expected in {in_s:.1} s, touched in {real:.1} s");
    }

    #[test]
    fn a_dive_at_altitude_is_not_a_landing() {
        let mut d = Detector::default();
        for i in 0..300 {
            let agl = 3000.0 - 40.0 * i as f64 * DT;
            assert_eq!(d.update(&subject(agl, 40.0, false, Systems::default()), DT, 3.0), None);
        }
    }

    #[test]
    fn reversers_report_when_they_open() {
        let mut d = Detector::default();
        let mut n = 0;
        for r in [0.0, 0.0, 0.05, 0.2, 0.6, 1.0, 1.0, 0.5, 0.0, 0.0, 0.3, 1.0] {
            n += d.update(&subject(3.0, 0.0, true, Systems { reverser: r, ..Systems::default() }), DT, 3.0).is_some() as usize;
        }
        assert_eq!(n, 2);
    }
}
