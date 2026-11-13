//! Conversions between MSFS and the director's frame.
//!
//! The director works in X-Plane's local frame: meters, +x east, +y up,
//! +z south; attitudes with heading clockwise from true north, pitch nose-up
//! positive and roll right-wing-down positive. Body frame: +x right, +y up,
//! +z aft.
//!
//! MSFS reports latitude/longitude, altitude in meters, and (in SimConnect)
//! pitch nose-down positive and bank left-wing-down positive. Its body frame
//! is +x right, +y up, +z forward.

use camship_core::{Attitude, Vec3};

const EARTH_RADIUS_M: f64 = 6_371_000.0;
/// Re-centre the local frame when the aircraft is this far from its origin,
/// so the flat-earth approximation stays exact to well under a meter and a
/// tenth of a degree around the aircraft.
pub const RECENTER_M: f64 = 5_000.0;

/// A flat local frame tangent at `origin` (latitude, longitude, degrees).
/// Altitude is used as-is for +y, so "up" follows the curve of the earth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalFrame {
    pub lat0: f64,
    pub lon0: f64,
}

impl LocalFrame {
    pub fn new(lat0: f64, lon0: f64) -> Self {
        Self { lat0, lon0 }
    }

    fn m_per_deg_lat() -> f64 {
        EARTH_RADIUS_M.to_radians()
    }

    fn m_per_deg_lon(self) -> f64 {
        EARTH_RADIUS_M.to_radians() * self.lat0.to_radians().cos().max(1e-6)
    }

    pub fn to_local(self, lat: f64, lon: f64, alt: f64) -> Vec3 {
        let dlon = wrap180(lon - self.lon0);
        Vec3::new(dlon * self.m_per_deg_lon(), alt, -(lat - self.lat0) * Self::m_per_deg_lat())
    }

    /// Latitude, longitude (degrees) and altitude (meters) of a local point.
    pub fn to_world(self, p: Vec3) -> (f64, f64, f64) {
        let lat = self.lat0 - p.z / Self::m_per_deg_lat();
        let lon = wrap180(self.lon0 + p.x / self.m_per_deg_lon());
        (lat, lon, p.y)
    }
}

fn wrap180(a: f64) -> f64 {
    (a + 180.0).rem_euclid(360.0) - 180.0
}

/// MSFS world velocity (+x east, +y up, +z north) to the local frame.
pub fn velocity(east: f64, up: f64, north: f64) -> Vec3 {
    Vec3::new(east, up, -north)
}

/// SimConnect's attitude (degrees) to the director's.
pub fn attitude(heading_true: f64, msfs_pitch: f64, msfs_bank: f64) -> Attitude {
    Attitude::new(heading_true, -msfs_pitch, -msfs_bank)
}

/// A director body-frame vector to MSFS's body frame.
pub fn body_to_msfs(v: Vec3) -> [f64; 3] {
    [v.x, v.y, -v.z]
}

/// Field of view for X-Plane-style zoom (1 = the user's normal view):
/// zoom divides tan(fov / 2). Degrees in, degrees out.
pub fn zoomed_fov(base_deg: f64, zoom: f64) -> f64 {
    let half = (base_deg / 2.0).to_radians().tan() / zoom.max(0.05);
    (2.0 * half.atan()).to_degrees().clamp(1.0, 170.0)
}

/// The sun's true azimuth and elevation (degrees) for a UTC time, accurate to
/// a fraction of a degree: plenty to frame a silhouette.
pub fn sun_position(day_of_year: f64, utc_seconds: f64, lat: f64, lon: f64) -> (f64, f64) {
    let b = (360.0 / 365.0 * (day_of_year - 81.0)).to_radians();
    let declination = 23.44f64.to_radians() * b.sin();
    // Equation of time, minutes.
    let eot = 9.87 * (2.0 * b).sin() - 7.53 * b.cos() - 1.5 * b.sin();
    let solar_minutes = utc_seconds / 60.0 + 4.0 * lon + eot;
    let hour_angle = (solar_minutes / 4.0 - 180.0).to_radians();
    let phi = lat.to_radians();
    let sin_el = phi.sin() * declination.sin() + phi.cos() * declination.cos() * hour_angle.cos();
    let elevation = sin_el.clamp(-1.0, 1.0).asin();
    let azimuth = (-hour_angle.sin()).atan2(declination.tan() * phi.cos() - phi.sin() * hour_angle.cos());
    (azimuth.to_degrees().rem_euclid(360.0), elevation.to_degrees())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_round_trip() {
        let f = LocalFrame::new(47.45, -122.3);
        let p = f.to_local(47.46, -122.28, 120.0);
        assert!(p.x > 1400.0 && p.x < 1600.0, "{p:?}"); // east
        assert!(p.z < -1000.0 && p.z > -1200.0, "{p:?}"); // north is -z
        let (lat, lon, alt) = f.to_world(p);
        assert!((lat - 47.46).abs() < 1e-9 && (lon + 122.28).abs() < 1e-9 && (alt - 120.0).abs() < 1e-9);
    }

    #[test]
    fn crosses_the_date_line() {
        let f = LocalFrame::new(0.0, 179.99);
        let p = f.to_local(0.0, -179.99, 0.0);
        assert!((p.x - 0.02 * EARTH_RADIUS_M.to_radians()).abs() < 1.0, "{p:?}");
        let (_, lon, _) = f.to_world(p);
        assert!((lon + 179.99).abs() < 1e-9);
    }

    #[test]
    fn msfs_attitude_signs() {
        // Nose up and banked right in MSFS terms: negative pitch, negative bank.
        let a = attitude(90.0, -10.0, -20.0);
        let nose = a.rotate(Vec3::FORWARD);
        assert!(nose.x > 0.9 && nose.y > 0.1, "nose east and up: {nose:?}");
        let right_wing = a.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(right_wing.y < -0.3, "right wing down: {right_wing:?}");
    }

    #[test]
    fn zoom_narrows_the_view() {
        assert!((zoomed_fov(60.0, 1.0) - 60.0).abs() < 1e-9);
        let z2 = zoomed_fov(60.0, 2.0);
        assert!((z2 - 32.2).abs() < 0.1, "{z2}");
    }

    #[test]
    fn sun_rises_in_the_east_and_peaks_at_noon() {
        // Equator, equinox: overhead at local noon, due east at 6 am.
        let (_, noon) = sun_position(81.0, 12.0 * 3600.0, 0.0, 0.0);
        assert!(noon > 80.0, "{noon}");
        let (az, el) = sun_position(81.0, 6.0 * 3600.0, 0.0, 0.0);
        assert!((az - 90.0).abs() < 5.0 && el.abs() < 5.0, "{az} {el}");
        // Paris, midsummer evening: low in the north-west.
        let (az, el) = sun_position(172.0, 19.5 * 3600.0, 48.85, 2.35);
        assert!((280.0..320.0).contains(&az) && (0.0..10.0).contains(&el), "{az} {el}");
    }
}
