//! Vectors, attitudes and smoothing in X-Plane's local OpenGL frame.
//!
//! World axes (X-Plane "local" coordinates): +x east, +y up, +z south.
//! Body axes used for shot offsets: +x right wing, +y up, +z aft (so the nose
//! points along -z, like the world frame at heading 0).

use std::f64::consts::PI;
use std::ops::{Add, AddAssign, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    pub const UP: Self = Self::new(0.0, 1.0, 0.0);
    /// Straight ahead in the body frame.
    pub const FORWARD: Self = Self::new(0.0, 0.0, -1.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn len(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn normalized_or(self, fallback: Self) -> Self {
        let l = self.len();
        if l > 1e-9 && l.is_finite() {
            self * (1.0 / l)
        } else {
            fallback
        }
    }

    pub fn lerp(self, o: Self, t: f64) -> Self {
        self + (o - self) * t
    }

    pub fn horizontal(self) -> Self {
        Self::new(self.x, 0.0, self.z)
    }

    pub fn mirror_x(self, m: f64) -> Self {
        Self::new(self.x * m, self.y, self.z)
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, o: Self) {
        *self = *self + o;
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k, self.z * k)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Wraps an angle in degrees to [-180, 180).
pub fn wrap180(a: f64) -> f64 {
    (a + 180.0).rem_euclid(360.0) - 180.0
}

/// Sine ease-in/ease-out on [0, 1]: a dolly that starts and stops gently.
pub fn ease_in_out(t: f64) -> f64 {
    0.5 - 0.5 * (PI * t.clamp(0.0, 1.0)).cos()
}

/// Frame-rate independent blend factor for an exponential low-pass.
pub fn exp_alpha(dt: f64, tau: f64) -> f64 {
    if tau <= 0.0 {
        1.0
    } else {
        1.0 - (-dt / tau).exp()
    }
}

/// Euler attitude in degrees, X-Plane conventions: heading clockwise from
/// true north, pitch nose-up positive, roll right-wing-down positive.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Attitude {
    pub heading: f64,
    pub pitch: f64,
    pub roll: f64,
}

impl Attitude {
    pub const fn new(heading: f64, pitch: f64, roll: f64) -> Self {
        Self { heading, pitch, roll }
    }

    /// Body-frame vector to world frame: roll, then pitch, then heading.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let (sr, cr) = (-self.roll).to_radians().sin_cos();
        let (sp, cp) = self.pitch.to_radians().sin_cos();
        let (sh, ch) = (-self.heading).to_radians().sin_cos();
        // Roll about z.
        let a = Vec3::new(v.x * cr - v.y * sr, v.x * sr + v.y * cr, v.z);
        // Pitch about x.
        let b = Vec3::new(a.x, a.y * cp - a.z * sp, a.y * sp + a.z * cp);
        // Heading about y.
        Vec3::new(b.x * ch + b.z * sh, b.y, -b.x * sh + b.z * ch)
    }

    /// World-frame vector to body frame: the inverse of `rotate`.
    pub fn unrotate(self, v: Vec3) -> Vec3 {
        // `rotate` is orthonormal, so its inverse is its transpose: the body
        // components are the projections on the rotated body axes.
        Vec3::new(
            v.dot(self.rotate(Vec3::new(1.0, 0.0, 0.0))),
            v.dot(self.rotate(Vec3::new(0.0, 1.0, 0.0))),
            v.dot(self.rotate(Vec3::new(0.0, 0.0, 1.0))),
        )
    }

    /// Same heading, wings level, nose on the horizon.
    pub fn level(self) -> Self {
        Self::new(self.heading, 0.0, 0.0)
    }

    /// World attitude of something mounted at `local` attitude in this body
    /// frame (e.g. a camera bolted to the wing).
    pub fn compose(self, local: Attitude) -> Attitude {
        let fwd = self.rotate(local.rotate(Vec3::FORWARD));
        let up = self.rotate(local.rotate(Vec3::UP));
        let (heading, pitch) = look_angles(fwd);
        let unrolled = Attitude::new(heading, pitch, 0.0);
        let (up0, right0) = (unrolled.rotate(Vec3::UP), unrolled.rotate(Vec3::new(1.0, 0.0, 0.0)));
        // Rolling right (X-Plane's positive roll) by r tilts the up vector to
        // cos r * up0 + sin r * right0.
        let roll = up.dot(right0).atan2(up.dot(up0)).to_degrees();
        Attitude::new(heading, pitch, roll)
    }
}

/// Heading and pitch (degrees) of a world-frame direction.
pub fn look_angles(dir: Vec3) -> (f64, f64) {
    let heading = dir.x.atan2(-dir.z).to_degrees().rem_euclid(360.0);
    let pitch = dir.y.atan2(dir.x.hypot(dir.z)).to_degrees();
    (heading, pitch)
}

/// Low-passed attitude: how a formation pilot in the camera ship follows the
/// subject's manoeuvres, a beat late.
#[derive(Clone, Copy, Debug, Default)]
pub struct SmoothAttitude {
    value: Option<Attitude>,
}

impl SmoothAttitude {
    pub fn reset(&mut self) {
        self.value = None;
    }

    pub fn update(&mut self, target: Attitude, dt: f64, tau: f64) -> Attitude {
        let next = match self.value {
            None => target,
            Some(v) => {
                let a = exp_alpha(dt, tau);
                Attitude::new(
                    (v.heading + wrap180(target.heading - v.heading) * a).rem_euclid(360.0),
                    v.pitch + wrap180(target.pitch - v.pitch) * a,
                    v.roll + wrap180(target.roll - v.roll) * a,
                )
            }
        };
        self.value = Some(next);
        next
    }
}

/// Critically damped spring on a vector: the camera operator's hands. Exact
/// closed-form step, so it is stable at any frame rate.
#[derive(Clone, Copy, Debug, Default)]
pub struct Spring3 {
    pos: Vec3,
    vel: Vec3,
    ready: bool,
}

impl Spring3 {
    pub fn is_ready(&self) -> bool {
        self.ready
    }

    /// The frame moved by `d` (new = old - d).
    pub fn shift(&mut self, d: Vec3) {
        self.pos = self.pos - d;
    }

    pub fn snap(&mut self, target: Vec3) {
        self.pos = target;
        self.vel = Vec3::ZERO;
        self.ready = true;
    }

    pub fn update(&mut self, target: Vec3, dt: f64, omega: f64) -> Vec3 {
        if !self.ready {
            self.snap(target);
            return target;
        }
        let x = self.pos - target;
        let decay = (-omega * dt).exp();
        let temp = (self.vel + x * omega) * dt;
        self.vel = (self.vel - temp * omega) * decay;
        self.pos = target + (x + temp) * decay;
        self.pos
    }
}

fn splitmix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// xorshift64*: small, deterministic, good enough to pick shots.
#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(splitmix(seed) | 1)
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in [0, 1).
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) -> bool {
        (a - b).len() < 1e-9
    }

    #[test]
    fn unrotate_inverts_rotate() {
        let att = Attitude::new(237.0, 12.0, -31.0);
        let v = Vec3::new(3.0, -2.0, 7.5);
        assert!(close(att.unrotate(att.rotate(v)), v));
        assert!(close(att.rotate(att.unrotate(v)), v));
    }

    #[test]
    fn nose_points_along_heading() {
        let east = Attitude::new(90.0, 0.0, 0.0).rotate(Vec3::FORWARD);
        assert!(close(east, Vec3::new(1.0, 0.0, 0.0)), "{east:?}");
        let south = Attitude::new(180.0, 0.0, 0.0).rotate(Vec3::FORWARD);
        assert!(close(south, Vec3::new(0.0, 0.0, 1.0)), "{south:?}");
    }

    #[test]
    fn pitch_up_raises_nose_and_roll_right_drops_right_wing() {
        let nose = Attitude::new(0.0, 30.0, 0.0).rotate(Vec3::FORWARD);
        assert!(nose.y > 0.49 && nose.y < 0.51);
        let wing = Attitude::new(0.0, 0.0, 30.0).rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(wing.y < -0.49 && wing.y > -0.51);
    }

    #[test]
    fn compose_matches_nested_rotation() {
        let body = Attitude::new(130.0, 12.0, -25.0);
        let local = Attitude::new(-80.0, -5.0, 0.0);
        let world = body.compose(local);
        for v in [Vec3::FORWARD, Vec3::UP, Vec3::new(1.0, 0.0, 0.0)] {
            let a = world.rotate(v);
            let b = body.rotate(local.rotate(v));
            assert!(close(a, b) || (a - b).len() < 1e-9, "{v:?}: {a:?} vs {b:?}");
        }
    }

    #[test]
    fn look_angles_invert_rotate() {
        for &(h, p) in &[(0.0, 0.0), (45.0, 10.0), (200.0, -35.0), (359.0, 80.0)] {
            let dir = Attitude::new(h, p, 0.0).rotate(Vec3::FORWARD);
            let (h2, p2) = look_angles(dir);
            assert!(wrap180(h2 - h).abs() < 1e-6, "{h} -> {h2}");
            assert!((p2 - p).abs() < 1e-6);
        }
    }

    #[test]
    fn wrap180_range() {
        assert_eq!(wrap180(190.0), -170.0);
        assert_eq!(wrap180(-190.0), 170.0);
        assert_eq!(wrap180(360.0), 0.0);
    }

    #[test]
    fn smooth_attitude_takes_short_way_round() {
        let mut s = SmoothAttitude::default();
        s.update(Attitude::new(359.0, 0.0, 0.0), 0.0, 1.0);
        let a = s.update(Attitude::new(1.0, 0.0, 0.0), 0.5, 1.0);
        assert!(a.heading > 359.0 || a.heading < 1.0, "{}", a.heading);
    }

    #[test]
    fn spring_settles_on_target() {
        let mut s = Spring3::default();
        s.snap(Vec3::ZERO);
        let target = Vec3::new(1.0, 2.0, 3.0);
        let mut p = Vec3::ZERO;
        for _ in 0..600 {
            p = s.update(target, 1.0 / 60.0, 6.0);
        }
        assert!(close(p, target) || (p - target).len() < 1e-6);
    }
}
