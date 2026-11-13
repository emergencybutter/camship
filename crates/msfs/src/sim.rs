//! The simulation variables Camship reads, and what they become for the
//! director.

use camship_core::{Subject, Systems, Vec3};

use crate::geo::{self, LocalFrame};

/// One SimConnect variable, read as FLOAT64.
pub struct Var {
    pub name: &'static str,
    pub units: &'static str,
}

const fn v(name: &'static str, units: &'static str) -> Var {
    Var { name, units }
}

/// Indices into `VARS`.
#[derive(Clone, Copy)]
#[repr(usize)]
enum F {
    Lat,
    Lon,
    Alt,
    Agl,
    GroundAlt,
    OnGround,
    Heading,
    Pitch,
    Bank,
    VelEast,
    VelUp,
    VelNorth,
    WingSpan,
    Gear,
    Flaps,
    Spoilers,
    LandingLight,
    Canopy,
    Exit0,
    Engines,
    Combustion1,
    Combustion2,
    Combustion3,
    Combustion4,
    Reverser1,
    Reverser2,
    Reverser3,
    Reverser4,
    ZuluTime,
    ZuluDay,
}

pub const VARS: &[Var] = &[
    v("PLANE LATITUDE", "degrees"),
    v("PLANE LONGITUDE", "degrees"),
    v("PLANE ALTITUDE", "meters"),
    v("PLANE ALT ABOVE GROUND", "meters"),
    v("GROUND ALTITUDE", "meters"),
    v("SIM ON GROUND", "bool"),
    v("PLANE HEADING DEGREES TRUE", "degrees"),
    v("PLANE PITCH DEGREES", "degrees"),
    v("PLANE BANK DEGREES", "degrees"),
    v("VELOCITY WORLD X", "meters per second"),
    v("VELOCITY WORLD Y", "meters per second"),
    v("VELOCITY WORLD Z", "meters per second"),
    v("WING SPAN", "meters"),
    v("GEAR TOTAL PCT EXTENDED", "percent over 100"),
    v("TRAILING EDGE FLAPS LEFT PERCENT", "percent over 100"),
    v("SPOILERS HANDLE POSITION", "percent over 100"),
    v("LIGHT LANDING", "bool"),
    v("CANOPY OPEN", "percent over 100"),
    v("EXIT OPEN:0", "percent over 100"),
    v("NUMBER OF ENGINES", "number"),
    v("GENERAL ENG COMBUSTION:1", "bool"),
    v("GENERAL ENG COMBUSTION:2", "bool"),
    v("GENERAL ENG COMBUSTION:3", "bool"),
    v("GENERAL ENG COMBUSTION:4", "bool"),
    v("TURB ENG REVERSE NOZZLE PERCENT:1", "percent over 100"),
    v("TURB ENG REVERSE NOZZLE PERCENT:2", "percent over 100"),
    v("TURB ENG REVERSE NOZZLE PERCENT:3", "percent over 100"),
    v("TURB ENG REVERSE NOZZLE PERCENT:4", "percent over 100"),
    v("ZULU TIME", "seconds"),
    v("ZULU DAY OF YEAR", "number"),
];

/// The latest values, with the variables the simulator rejected marked
/// missing. A rejected variable is left out of the data it sends, so the
/// received values are spread over the accepted ones in order.
#[derive(Clone, Debug)]
pub struct Sample {
    values: Vec<Option<f64>>,
}

impl Sample {
    /// `accepted[i]`: whether `VARS[i]` is in the data definition.
    pub fn from_values(values: &[f64], accepted: &[bool]) -> Option<Self> {
        let mut it = values.iter();
        let mut out = Vec::with_capacity(VARS.len());
        for &ok in accepted {
            out.push(if ok { Some(*it.next()?).filter(|v| v.is_finite()) } else { None });
        }
        Some(Self { values: out })
    }

    fn get(&self, f: F) -> Option<f64> {
        self.values.get(f as usize).copied().flatten()
    }

    fn or(&self, f: F, default: f64) -> f64 {
        self.get(f).unwrap_or(default)
    }

    /// Latitude, longitude, altitude, if known.
    pub fn position(&self) -> Option<(f64, f64, f64)> {
        Some((self.get(F::Lat)?, self.get(F::Lon)?, self.get(F::Alt)?))
    }

    pub fn ground_altitude(&self) -> Option<f64> {
        self.get(F::GroundAlt)
    }

    pub fn wing_span(&self) -> Option<f64> {
        self.get(F::WingSpan).filter(|s| *s > 1.0)
    }

    /// The sun's true azimuth and elevation, degrees.
    pub fn sun(&self) -> Option<(f64, f64)> {
        let (lat, lon, _) = self.position()?;
        Some(geo::sun_position(self.get(F::ZuluDay)?, self.get(F::ZuluTime)?, lat, lon))
    }

    pub fn subject(&self, frame: &LocalFrame, size: f64) -> Option<Subject> {
        let (lat, lon, alt) = self.position()?;
        Some(Subject {
            pos: frame.to_local(lat, lon, alt),
            vel: geo::velocity(self.or(F::VelEast, 0.0), self.or(F::VelUp, 0.0), self.or(F::VelNorth, 0.0)),
            att: geo::attitude(self.or(F::Heading, 0.0), self.or(F::Pitch, 0.0), self.or(F::Bank, 0.0)),
            agl: self.or(F::Agl, 1000.0),
            on_ground: self.or(F::OnGround, 0.0) != 0.0,
            size,
            systems: self.systems(),
        })
    }

    /// Missing variables read as gear down, flaps up and so on, so they never
    /// trigger anything.
    fn systems(&self) -> Systems {
        let d = Systems::default();
        let n = self.or(F::Engines, 4.0).clamp(0.0, 4.0) as usize;
        let combustion = [F::Combustion1, F::Combustion2, F::Combustion3, F::Combustion4];
        let reversers = [F::Reverser1, F::Reverser2, F::Reverser3, F::Reverser4];
        let engines_running = combustion
            .iter()
            .take(n)
            .enumerate()
            .filter(|(_, &f)| self.or(f, 0.0) != 0.0)
            .fold(0u16, |bits, (i, _)| bits | 1 << i);
        let reverser = reversers.iter().take(n).filter_map(|&f| self.get(f)).reduce(f64::max);
        let doors = [self.get(F::Canopy), self.get(F::Exit0)].into_iter().flatten().reduce(f64::max);
        Systems {
            gear: self.get(F::Gear).unwrap_or(d.gear),
            flaps: self.get(F::Flaps).unwrap_or(d.flaps),
            reverser: reverser.unwrap_or(d.reverser),
            speedbrake: self.get(F::Spoilers).unwrap_or(d.speedbrake),
            landing_lights: self.get(F::LandingLight).unwrap_or(d.landing_lights),
            doors: doors.unwrap_or(d.doors),
            engines_running,
        }
    }
}

/// Body-frame (director) offset of `p` from the subject.
pub fn body_offset(s: &Subject, p: Vec3) -> Vec3 {
    s.att.unrotate(p - s.pos)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejected_variables_are_skipped() {
        let mut accepted = vec![true; VARS.len()];
        accepted[F::Agl as usize] = false;
        let values: Vec<f64> = (0..VARS.len() - 1).map(|i| i as f64).collect();
        let s = Sample::from_values(&values, &accepted).unwrap();
        assert_eq!(s.get(F::Alt), Some(2.0));
        assert_eq!(s.get(F::Agl), None);
        assert_eq!(s.get(F::GroundAlt), Some(3.0));
        assert!(Sample::from_values(&values[..3], &accepted).is_none());
    }

    #[test]
    fn subject_from_sample() {
        let mut values = vec![0.0; VARS.len()];
        values[F::Lat as usize] = 10.0;
        values[F::Lon as usize] = 20.0;
        values[F::Alt as usize] = 500.0;
        values[F::VelNorth as usize] = 60.0;
        values[F::Engines as usize] = 2.0;
        values[F::Combustion2 as usize] = 1.0;
        values[F::Gear as usize] = 1.0;
        let s = Sample::from_values(&values, &vec![true; VARS.len()]).unwrap();
        let subj = s.subject(&LocalFrame::new(10.0, 20.0), 30.0).unwrap();
        assert!(subj.pos.len() < 1e-6 + 500.0 && (subj.pos.y - 500.0).abs() < 1e-9);
        assert_eq!(subj.vel, Vec3::new(0.0, 0.0, -60.0));
        assert_eq!(subj.systems.engines_running, 0b10);
    }
}
