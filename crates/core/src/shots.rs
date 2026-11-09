//! The shot list: what a camera ship and a ground crew would film for an
//! aircraft commercial.
//!
//! Every distance is in *subject sizes* (roughly the wingspan), so the same
//! shot frames a Cessna and a 747 alike. Offsets use the body frame: +x right
//! wing, +y up, +z aft. Each shot moves during its take (dolly, crane, orbit,
//! zoom creep); a static frame reads as a screenshot, not a commercial.

use crate::events::Trigger;
use crate::math::Vec3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Path {
    /// The camera ship glides from `from` to `to`, holding formation in the
    /// (slightly lagged) frame of the subject.
    Line { from: Vec3, to: Vec3 },
    /// An arc around the subject, kept level. Azimuth in degrees from the
    /// nose, clockwise seen from above (90 = off the right wing).
    Orbit { radius: f64, az_from: f64, az_to: f64, height_from: f64, height_to: f64 },
    /// A photographer standing still (on the ground, or a hovering ship) at a
    /// point ahead of the subject, beside its track, panning as it passes.
    /// With `on_terrain` they stand on the ground even when the subject is
    /// flying, e.g. under the approach.
    Flyby { side: f64, height: f64, on_terrain: bool },
    /// A camera on a tripod beside the track, like `Flyby`, but locked off:
    /// aimed once at the point where the subject will pass, it never pans.
    /// The aircraft enters the frame, crosses it and leaves.
    LockedOff { side: f64, height: f64 },
    /// Keeps the sun right behind the subject: the camera sits on the far
    /// side from the sun, `distance` sizes out, and swings `sweep_deg` either
    /// side of that line so the sun slides behind the aircraft mid-take.
    /// Only filmed when the sun is low (see the director).
    SunBehind { distance: f64, sweep_deg: f64 },
    /// A camera fixed to the airframe at `at` (in sizes, never distance-
    /// scaled) plus `offset_m` (meters, e.g. the length of a selfie stick),
    /// aimed rigidly at the look point: it moves, pitches and banks exactly
    /// with the aircraft.
    Mount { at: Vec3, offset_m: Vec3 },
    /// Directly above the subject on the world vertical, looking straight
    /// down, nose at the top of the frame. Height in sizes.
    Overhead { height_from: f64, height_to: f64 },
}

/// What the shot is built around.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Target {
    /// Offsets are from the aircraft's reference point, in subject sizes.
    Airframe,
    /// Offsets are from one engine, in nacelle sizes (see
    /// [`nacelle_size`]), with +x pointing outboard from that engine.
    Engine {
        /// Only engines clearly off the centreline (not a nose propeller,
        /// where "behind the engine" is inside the cockpit).
        outboard_only: bool,
    },
    /// Offsets are from one landing gear wheel, in wheel sizes (see
    /// [`wheel_size`]), with +x pointing outboard from that wheel.
    Gear {
        /// The nose gear instead of a main gear.
        nose: bool,
    },
}

/// Rough nacelle / propeller diameter for an aircraft `size` meters across:
/// about 1.2 m on a Cessna, 2 m on an A320, 2.8 m on a 747.
pub fn nacelle_size(size: f64) -> f64 {
    0.35 * size.max(1.0).sqrt()
}

/// Rough main wheel diameter for an aircraft `size` meters across: about
/// 0.6 m on a Cessna, 1 m on an A320, 1.4 m on a 747.
pub fn wheel_size(size: f64) -> f64 {
    0.18 * size.max(1.0).sqrt()
}

/// How shots are grouped in the settings window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Group {
    /// A camera plane flying with the subject.
    CameraShip,
    /// Tight shots on one part of the airframe.
    CloseUp,
    /// Mounted, overhead and fixed cameras.
    Special,
    /// Filmed only when something happens: gear, flaps, touchdown, reversers.
    Action,
}

impl Group {
    pub const ALL: [Group; 4] = [Group::CameraShip, Group::CloseUp, Group::Special, Group::Action];

    pub fn title(self) -> &'static str {
        match self {
            Group::CameraShip => "Camera plane",
            Group::CloseUp => "Close-ups",
            Group::Special => "Special cameras",
            Group::Action => "Action shots",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shot {
    /// Stable identifier, used in the prefs file.
    pub id: &'static str,
    pub name: &'static str,
    pub group: Group,
    /// One line for the settings window.
    pub blurb: &'static str,
    pub path: Path,
    pub target: Target,
    /// Point on the subject the lens is aimed at, at the start and end of the take.
    pub look_from: Vec3,
    pub look_to: Vec3,
    /// X-Plane zoom factor (1 = the user's normal field of view) at start and end.
    pub zoom: (f64, f64),
    /// A fixed horizontal field of view in degrees, whatever the user's own
    /// FOV setting is (an ultra-wide action camera). Replaces `zoom`.
    pub fov_deg: Option<f64>,
    /// Fixed camera roll in degrees, for a slightly tilted "Dutch" frame.
    pub dutch: f64,
    /// How much the camera banks with the subject (0 = horizon stays level).
    pub bank_follow: f64,
    pub in_air: bool,
    pub on_ground: bool,
    /// Required height above ground, in subject sizes.
    pub min_agl: f64,
    /// Only filmed below this height above ground, in subject sizes.
    pub max_agl: f64,
    /// Drone-style reveal: the take opens with the lens tilted this many
    /// degrees down from the aircraft, at the scenery, and tilts up to find
    /// it over the first part of the move.
    pub reveal_tilt: f64,
    /// Required ground speed, m/s.
    pub min_speed: f64,
    /// The director may play this shot from the other side.
    pub mirror: bool,
    /// Relative probability of being picked.
    pub weight: f64,
    /// Action shots: only filmed, straight away, when this happens. Never
    /// picked at random.
    pub trigger: Option<Trigger>,
}

const fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z)
}

const BASE: Shot = Shot {
    id: "",
    name: "",
    group: Group::CameraShip,
    blurb: "",
    path: Path::Line { from: Vec3::ZERO, to: Vec3::ZERO },
    target: Target::Airframe,
    look_from: Vec3::ZERO,
    look_to: Vec3::ZERO,
    zoom: (1.0, 1.0),
    fov_deg: None,
    dutch: 0.0,
    bank_follow: 0.5,
    in_air: true,
    on_ground: true,
    min_agl: 0.0,
    max_agl: f64::INFINITY,
    reveal_tilt: 0.0,
    min_speed: 0.0,
    mirror: true,
    weight: 1.0,
    trigger: None,
};

pub const CATALOG: &[Shot] = &[
    Shot {
        id: "rear_quarter",
        name: "Rear-quarter reveal",
        group: Group::CameraShip,
        blurb: "Slides up from behind the wing to abeam",
        path: Path::Line { from: v(-0.9, 0.12, 1.5), to: v(-1.2, 0.22, 0.35) },
        look_from: v(0.0, 0.0, 0.05),
        look_to: v(0.0, 0.0, -0.05),
        zoom: (1.15, 1.35),
        weight: 1.2,
        ..BASE
    },
    Shot {
        id: "abeam",
        name: "Formation abeam",
        group: Group::CameraShip,
        blurb: "Formation off the wing, sliding forward",
        path: Path::Line { from: v(1.6, 0.06, 0.7), to: v(1.6, 0.12, -0.7) },
        look_from: v(0.0, 0.0, 0.1),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.5, 1.6),
        bank_follow: 0.8,
        on_ground: false,
        min_agl: 0.5,
        ..BASE
    },
    Shot {
        id: "hero_nose",
        name: "Hero front quarter",
        group: Group::CameraShip,
        blurb: "Low ahead of the nose, slight tilt, push-in",
        path: Path::Line { from: v(0.6, -0.1, -1.4), to: v(0.85, -0.02, -1.15) },
        look_from: v(0.0, 0.0, -0.15),
        look_to: v(0.0, 0.0, -0.15),
        zoom: (1.25, 1.55),
        dutch: 5.0,
        bank_follow: 0.4,
        ..BASE
    },
    Shot {
        id: "head_on",
        name: "Head-on",
        group: Group::CameraShip,
        blurb: "Ahead, looking back at the nose",
        // Flying ahead of the nose, looking back, slowly letting it catch up.
        path: Path::Line { from: v(0.15, 0.12, -2.4), to: v(0.25, 0.08, -1.9) },
        look_from: v(0.0, 0.0, -0.2),
        look_to: v(0.0, 0.0, -0.2),
        zoom: (1.3, 1.45),
        bank_follow: 0.6,
        weight: 1.1,
        ..BASE
    },
    Shot {
        id: "overtake_nose",
        name: "Overtake across the nose",
        group: Group::CameraShip,
        blurb: "Overtakes and crosses ahead of the nose",
        // From behind the left wing, overtaking and crossing ahead to the right.
        path: Path::Line { from: v(-1.2, 0.1, 0.5), to: v(0.45, 0.1, -1.9) },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, -0.2),
        zoom: (1.2, 1.3),
        bank_follow: 0.6,
        weight: 1.0,
        ..BASE
    },
    Shot {
        id: "overtake_above",
        name: "Overtake above",
        group: Group::CameraShip,
        blurb: "Passes slowly over, from behind to ahead",
        // Passing over from behind to ahead; slightly off-centre so the lens
        // never points straight down.
        path: Path::Line { from: v(0.25, 0.5, 1.7), to: v(0.25, 0.55, -1.7) },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.15, 1.15),
        bank_follow: 0.6,
        on_ground: false,
        min_agl: 2.0,
        weight: 0.9,
        ..BASE
    },
    Shot {
        id: "wingtip",
        name: "Wingtip cam",
        group: Group::Special,
        blurb: "Insta360-style, just ahead of the wingtip",
        // An action camera on a stick a foot or two ahead of the right
        // wingtip's leading edge, ultra-wide, looking back in at the fuselage.
        // The tip is ~half a span out; the stick is in meters so it's the
        // same real length on every aircraft. From there the nose and tail
        // are up to ~55 degrees either side of the fuselage, so the lens
        // takes in 120 degrees to show the whole aircraft.
        path: Path::Mount { at: v(0.5, 0.03, -0.02), offset_m: v(0.0, 0.1, -1.1) },
        look_from: v(0.0, 0.06, 0.04),
        look_to: v(0.0, 0.06, 0.04),
        fov_deg: Some(120.0),
        bank_follow: 0.0,
        weight: 0.9,
        ..BASE
    },
    Shot {
        id: "fin_cam",
        name: "Fin-top cam",
        group: Group::Special,
        blurb: "On a mast above the fin, looking forward",
        // Like an airliner's tail camera: above the top of the fin, looking
        // forward along the whole fuselage to the horizon. The fin top is
        // 0.13 (light aircraft) to 0.22 (airliners) sizes up, so the mast
        // clears it on every type. Tilts slowly up from the nose.
        path: Path::Mount { at: v(0.0, 0.22, 0.4), offset_m: v(0.0, 0.4, 0.0) },
        look_from: v(0.0, 0.0, -0.55),
        look_to: v(0.0, 0.04, -0.7),
        fov_deg: Some(100.0),
        bank_follow: 0.0,
        mirror: false,
        weight: 0.7,
        ..BASE
    },
    Shot {
        id: "belly_cam",
        name: "Belly cam",
        group: Group::Special,
        blurb: "Under the nose, looking down the runway ahead",
        // Just under the forward fuselage (the belly is about 0.06 sizes
        // below the reference point), looking forward and down at the
        // centreline. Best taxiing, taking off and landing. It never goes
        // below the ground, whatever the aircraft's real belly height.
        path: Path::Mount { at: v(0.0, -0.06, -0.15), offset_m: v(0.0, -0.15, 0.0) },
        look_from: v(0.0, -0.1, -1.2),
        look_to: v(0.0, -0.08, -1.4),
        fov_deg: Some(100.0),
        bank_follow: 0.0,
        mirror: false,
        weight: 0.6,
        ..BASE
    },
    Shot {
        id: "shoulder",
        name: "Over the shoulder",
        group: Group::Special,
        blurb: "Above the wing root, looking forward past the cockpit",
        // Above the fuselage's shoulder, over the wing root, looking forward
        // along the nose to the horizon.
        path: Path::Mount { at: v(0.08, 0.13, 0.0), offset_m: v(0.0, 0.3, 0.0) },
        look_from: v(0.0, 0.02, -1.4),
        look_to: v(0.0, 0.03, -1.6),
        fov_deg: Some(90.0),
        bank_follow: 0.0,
        weight: 0.6,
        ..BASE
    },
    Shot {
        id: "straight_down",
        name: "Top-down",
        group: Group::Special,
        blurb: "Straight down, aircraft centred",
        path: Path::Overhead { height_from: 1.6, height_to: 2.1 },
        zoom: (1.0, 1.0),
        bank_follow: 0.0,
        mirror: false,
        weight: 0.8,
        ..BASE
    },
    Shot {
        id: "top_down",
        name: "Overhead pass",
        group: Group::CameraShip,
        blurb: "High above, aircraft overtakes the camera",
        path: Path::Line { from: v(0.1, 1.4, -0.9), to: v(0.1, 1.4, 1.1) },
        look_from: v(0.0, 0.0, -0.2),
        look_to: v(0.0, 0.0, 0.2),
        zoom: (1.1, 1.1),
        bank_follow: 0.7,
        on_ground: false,
        min_agl: 2.0,
        weight: 0.7,
        ..BASE
    },
    Shot {
        id: "orbit",
        name: "Slow orbit",
        group: Group::CameraShip,
        blurb: "Level arc around the aircraft",
        path: Path::Orbit { radius: 1.7, az_from: 210.0, az_to: 330.0, height_from: 0.25, height_to: 0.4 },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.2, 1.2),
        bank_follow: 0.3,
        ..BASE
    },
    Shot {
        id: "front_orbit",
        name: "Front orbit",
        group: Group::CameraShip,
        blurb: "Arc across the nose, from one side to the other",
        // From ahead of one wing, round in front of the nose, to ahead of the other.
        path: Path::Orbit { radius: 1.7, az_from: 300.0, az_to: 420.0, height_from: 0.2, height_to: 0.3 },
        look_from: v(0.0, 0.0, -0.1),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.2, 1.2),
        bank_follow: 0.3,
        weight: 0.9,
        ..BASE
    },
    Shot {
        id: "sun_silhouette",
        name: "Sun silhouette",
        group: Group::CameraShip,
        blurb: "Low sun right behind the aircraft (sunrise and sunset only)",
        // Telephoto from the far side of the sun; the sun slides behind the
        // aircraft halfway through. Rare (low sun only), so weighted up.
        path: Path::SunBehind { distance: 3.0, sweep_deg: 12.0 },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (2.0, 2.2),
        bank_follow: 0.3,
        weight: 1.5,
        ..BASE
    },
    Shot {
        id: "belly",
        name: "Under the wing",
        group: Group::CameraShip,
        blurb: "Looking up from under the wing",
        path: Path::Line { from: v(-0.5, -0.75, 1.0), to: v(-0.35, -0.6, 0.25) },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.15, 1.15),
        dutch: -3.0,
        bank_follow: 0.6,
        on_ground: false,
        min_agl: 3.0,
        weight: 0.6,
        ..BASE
    },
    Shot {
        id: "tail",
        name: "Tail detail",
        group: Group::CloseUp,
        blurb: "Tight on the fin",
        path: Path::Line { from: v(-0.55, 0.4, 1.0), to: v(-0.4, 0.32, 0.8) },
        look_from: v(0.0, 0.12, 0.42),
        look_to: v(0.0, 0.14, 0.4),
        zoom: (2.0, 2.4),
        dutch: 3.0,
        weight: 0.7,
        ..BASE
    },
    Shot {
        id: "engine",
        name: "Wing detail",
        group: Group::CloseUp,
        blurb: "Tight on the inboard wing",
        path: Path::Line { from: v(0.95, 0.12, -0.35), to: v(0.85, 0.15, 0.05) },
        look_from: v(0.28, -0.04, -0.08),
        look_to: v(0.3, -0.03, -0.04),
        zoom: (2.0, 2.3),
        dutch: -2.0,
        weight: 0.7,
        ..BASE
    },
    Shot {
        id: "engine_front",
        name: "Engine: intake",
        group: Group::CloseUp,
        blurb: "Engine intake or propeller, from ahead",
        path: Path::Line { from: v(1.2, -0.4, -3.6), to: v(0.8, -0.25, -2.8) },
        target: Target::Engine { outboard_only: false },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.2),
        zoom: (1.4, 1.55),
        dutch: 3.0,
        weight: 0.6,
        ..BASE
    },
    Shot {
        id: "engine_side",
        name: "Engine: nacelle side",
        group: Group::CloseUp,
        blurb: "Along the engine, from outboard",
        // Outboard and a little below, under the wing, sliding front to back.
        path: Path::Line { from: v(3.2, -0.5, -1.2), to: v(3.2, -0.4, 1.2) },
        target: Target::Engine { outboard_only: false },
        look_from: v(0.0, 0.0, -0.2),
        look_to: v(0.0, 0.0, 0.3),
        zoom: (1.4, 1.4),
        weight: 0.6,
        ..BASE
    },
    Shot {
        id: "engine_rear",
        name: "Engine: exhaust",
        group: Group::CloseUp,
        blurb: "Engine exhaust, from behind",
        path: Path::Line { from: v(1.0, -0.5, 3.6), to: v(0.7, -0.35, 2.8) },
        target: Target::Engine { outboard_only: true },
        look_from: v(0.0, 0.0, 0.2),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.4, 1.55),
        dutch: -3.0,
        weight: 0.5,
        ..BASE
    },
    Shot {
        id: "engine_below",
        name: "Engine: from below",
        group: Group::CloseUp,
        blurb: "Looking up at the engine",
        path: Path::Line { from: v(0.6, -3.0, -1.0), to: v(0.8, -2.6, 0.8) },
        target: Target::Engine { outboard_only: false },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.4, 1.4),
        on_ground: false,
        min_agl: 1.0,
        weight: 0.4,
        ..BASE
    },
    Shot {
        id: "long_lens",
        name: "Long-lens chase",
        group: Group::CameraShip,
        blurb: "Far behind, long telephoto",
        path: Path::Line { from: v(0.25, 0.08, 6.0), to: v(0.3, 0.14, 5.0) },
        look_from: v(0.0, 0.0, -0.1),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (4.0, 4.6),
        bank_follow: 0.8,
        weight: 0.8,
        ..BASE
    },
    Shot {
        id: "crane",
        name: "Crane up from behind",
        group: Group::CameraShip,
        blurb: "Rises from tail height to a high rear view",
        path: Path::Line { from: v(0.0, -0.02, 1.7), to: v(0.05, 0.85, 1.25) },
        look_from: v(0.0, 0.0, -0.3),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.2, 1.2),
        bank_follow: 0.3,
        weight: 0.7,
        ..BASE
    },
    Shot {
        id: "dolly_zoom",
        name: "Dolly zoom",
        group: Group::CameraShip,
        blurb: "Backs away while zooming in: the background swells",
        // The Vertigo effect: the camera triples its distance along one line
        // through the aim point while the zoom triples, so the aircraft stays
        // the same size and the scenery behind it grows.
        path: Path::Line { from: v(0.75, 0.06, -0.45), to: v(2.25, 0.18, -1.35) },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.0, 3.0),
        weight: 0.8,
        ..BASE
    },
    Shot {
        id: "breakaway",
        name: "Breakaway",
        group: Group::CameraShip,
        blurb: "Camera plane drops back and away",
        // How an air-to-air sortie ends: from formation off the wing, the
        // camera ship eases off its power and descends away, and the aircraft
        // pulls ahead and shrinks in the frame.
        path: Path::Line { from: v(1.3, 0.05, 0.1), to: v(2.6, -0.35, 3.2) },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.4, 1.5),
        bank_follow: 0.7,
        on_ground: false,
        min_agl: 0.6,
        weight: 0.7,
        ..BASE
    },
    Shot {
        id: "flyby",
        name: "Flyby",
        group: Group::Special,
        blurb: "Photographer by the track, aircraft passes",
        path: Path::Flyby { side: 1.8, height: 0.15, on_terrain: false },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.5, 1.3),
        bank_follow: 0.0,
        min_speed: 25.0,
        weight: 0.8,
        ..BASE
    },
    Shot {
        id: "planespotter",
        name: "Planespotter",
        group: Group::Special,
        blurb: "Long lens from far off the track",
        // A spotter at the airport fence with a 600 mm lens: far to the side,
        // heavy compression, a slow pan as the aircraft goes by.
        path: Path::Flyby { side: 6.0, height: 0.1, on_terrain: false },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (4.5, 3.5),
        bank_follow: 0.0,
        min_speed: 25.0,
        weight: 0.6,
        ..BASE
    },
    Shot {
        id: "under_approach",
        name: "Under the approach",
        group: Group::Special,
        blurb: "Standing under the flight path as it passes overhead (low only)",
        // On the ground almost right under the track, looking up as the
        // aircraft roars over: on final or just after takeoff.
        path: Path::Flyby { side: 0.3, height: 0.0, on_terrain: true },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.0, 1.0),
        bank_follow: 0.0,
        on_ground: false,
        max_agl: 4.0,
        min_speed: 25.0,
        weight: 1.2,
        ..BASE
    },
    Shot {
        id: "drone_reveal",
        name: "Drone reveal",
        group: Group::Special,
        blurb: "Starts on the scenery, tilts up to find the aircraft",
        // A drone ahead and to the side, rising, its lens tilting up from the
        // landscape until the aircraft comes into frame.
        path: Path::Line { from: v(1.0, -0.1, -1.9), to: v(0.9, 0.3, -1.5) },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.0, 1.1),
        bank_follow: 0.2,
        reveal_tilt: 50.0,
        weight: 0.7,
        ..BASE
    },
    Shot {
        id: "locked_off",
        name: "Locked-off frame",
        group: Group::Special,
        blurb: "Fixed camera, no pan: the aircraft crosses the frame",
        path: Path::LockedOff { side: 2.2, height: 0.05 },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.0, 1.0),
        bank_follow: 0.0,
        min_speed: 25.0,
        weight: 0.6,
        ..BASE
    },
    // Action shots: never picked at random, cut to the moment it happens.
    Shot {
        id: "gear_below",
        name: "Gear: from below",
        group: Group::Action,
        blurb: "Gear moving: under the wing, looking up at the main gear",
        // Below, outboard and ahead of a main wheel, drifting aft as the
        // gear swings down out of (or up into) its bay.
        path: Path::Line { from: v(2.6, -3.4, -2.8), to: v(2.9, -2.9, 0.4) },
        target: Target::Gear { nose: false },
        look_from: v(0.0, 0.6, 0.0),
        look_to: v(0.0, 0.4, 0.1),
        zoom: (1.2, 1.3),
        dutch: 3.0,
        on_ground: false,
        min_agl: 0.5,
        trigger: Some(Trigger::Gear),
        ..BASE
    },
    Shot {
        id: "gear_nose",
        name: "Gear: nose gear",
        group: Group::Action,
        blurb: "Gear moving: ahead and below the nose gear",
        path: Path::Line { from: v(1.4, -3.0, -5.0), to: v(1.0, -2.4, -4.0) },
        target: Target::Gear { nose: true },
        look_from: v(0.0, 0.6, 0.0),
        look_to: v(0.0, 0.4, 0.0),
        zoom: (1.3, 1.4),
        on_ground: false,
        min_agl: 0.5,
        trigger: Some(Trigger::Gear),
        ..BASE
    },
    Shot {
        id: "gear_cam",
        name: "Gear: wheel-well cam",
        group: Group::Action,
        blurb: "Gear moving: rigid camera under the wing, looking back at the main gear",
        // On a short stick under the wing, outboard of and ahead of the
        // wheel, looking back up into the bay.
        path: Path::Mount { at: v(1.2, -0.3, -1.8), offset_m: Vec3::ZERO },
        target: Target::Gear { nose: false },
        look_from: v(0.0, 0.6, 0.2),
        look_to: v(0.0, 0.5, 0.2),
        fov_deg: Some(90.0),
        bank_follow: 0.0,
        trigger: Some(Trigger::Gear),
        ..BASE
    },
    Shot {
        id: "flaps_window",
        name: "Flaps: window seat",
        group: Group::Action,
        blurb: "Flaps moving: from the cabin, looking out at the trailing edge",
        // A passenger's view: just outside the fuselage above the wing root,
        // looking out and down at the inboard flap.
        path: Path::Mount { at: v(0.075, 0.03, 0.12), offset_m: v(0.35, 0.0, 0.0) },
        look_from: v(0.3, -0.03, 0.07),
        look_to: v(0.3, -0.03, 0.08),
        fov_deg: Some(75.0),
        bank_follow: 0.0,
        trigger: Some(Trigger::Flaps),
        ..BASE
    },
    Shot {
        id: "flaps_behind",
        name: "Flaps: trailing edge",
        group: Group::Action,
        blurb: "Flaps moving: from behind and above the wing",
        path: Path::Line { from: v(0.6, 0.25, 0.95), to: v(0.5, 0.2, 0.65) },
        look_from: v(0.3, -0.03, 0.06),
        look_to: v(0.28, -0.03, 0.05),
        zoom: (1.6, 1.8),
        dutch: 2.0,
        trigger: Some(Trigger::Flaps),
        ..BASE
    },
    Shot {
        id: "touchdown_runway",
        name: "Touchdown: runway edge",
        group: Group::Action,
        blurb: "Landing: standing at the runway edge where the wheels touch",
        // A photographer crouched by the runway at the touchdown point,
        // tight on the main gear as it meets the ground.
        path: Path::Flyby { side: 1.1, height: 0.0, on_terrain: true },
        target: Target::Gear { nose: false },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (2.4, 2.0),
        bank_follow: 0.0,
        trigger: Some(Trigger::Touchdown),
        ..BASE
    },
    Shot {
        id: "touchdown_gear_cam",
        name: "Touchdown: gear cam",
        group: Group::Action,
        blurb: "Landing: rigid camera beside the main wheels as they hit the runway",
        path: Path::Mount { at: v(1.0, 0.2, -1.6), offset_m: Vec3::ZERO },
        target: Target::Gear { nose: false },
        look_from: v(0.0, -0.5, 0.3),
        look_to: v(0.0, -0.5, 0.3),
        fov_deg: Some(85.0),
        bank_follow: 0.0,
        trigger: Some(Trigger::Touchdown),
        ..BASE
    },
    Shot {
        id: "touchdown_chase",
        name: "Touchdown: low chase",
        group: Group::Action,
        blurb: "Landing: low behind the main gear, following it onto the runway",
        path: Path::Line { from: v(2.0, -0.2, 4.0), to: v(2.3, 0.1, 6.0) },
        target: Target::Gear { nose: false },
        look_from: v(0.0, -0.3, 0.0),
        look_to: v(0.0, -0.3, 0.0),
        zoom: (1.7, 1.9),
        bank_follow: 0.2,
        trigger: Some(Trigger::Touchdown),
        ..BASE
    },
    Shot {
        id: "reverser_side",
        name: "Reverser: nacelle",
        group: Group::Action,
        blurb: "Reversers opening: outboard of the engine, on the aft nacelle",
        path: Path::Line { from: v(3.0, -0.3, -0.6), to: v(3.0, -0.2, 0.5) },
        target: Target::Engine { outboard_only: false },
        look_from: v(0.0, 0.0, 0.5),
        look_to: v(0.0, 0.0, 0.7),
        zoom: (1.4, 1.5),
        trigger: Some(Trigger::Reverser),
        ..BASE
    },
    Shot {
        id: "reverser_behind",
        name: "Reverser: from behind",
        group: Group::Action,
        blurb: "Reversers opening: low behind the engine, looking forward into the open reverser",
        // Behind and outboard, closing in. Works for engines under the wing
        // and on the rear fuselage alike.
        path: Path::Line { from: v(2.2, -0.4, 4.0), to: v(1.8, -0.3, 3.0) },
        target: Target::Engine { outboard_only: false },
        look_from: v(0.0, 0.0, 0.6),
        look_to: v(0.0, 0.0, 0.5),
        zoom: (1.3, 1.4),
        trigger: Some(Trigger::Reverser),
        ..BASE
    },
    Shot {
        id: "rotation_runway",
        name: "Rotation: runway edge",
        group: Group::Action,
        blurb: "Takeoff: by the runway as the nose comes up and the wheels leave",
        // A photographer at the runway edge just ahead: the aircraft passes
        // as it lifts off.
        path: Path::Flyby { side: 1.3, height: 0.0, on_terrain: true },
        look_from: v(0.0, 0.0, 0.05),
        look_to: v(0.0, 0.0, 0.05),
        zoom: (1.7, 1.3),
        bank_follow: 0.0,
        trigger: Some(Trigger::Rotation),
        ..BASE
    },
    Shot {
        id: "rotation_gear_cam",
        name: "Rotation: gear cam",
        group: Group::Action,
        blurb: "Takeoff: rigid camera beside the main wheels as they leave the runway",
        path: Path::Mount { at: v(1.0, 0.2, -1.6), offset_m: Vec3::ZERO },
        target: Target::Gear { nose: false },
        look_from: v(0.0, -0.5, 0.3),
        look_to: v(0.0, -0.6, 0.3),
        fov_deg: Some(85.0),
        bank_follow: 0.0,
        trigger: Some(Trigger::Rotation),
        ..BASE
    },
    Shot {
        id: "rotation_behind",
        name: "Rotation: chase",
        group: Group::Action,
        blurb: "Takeoff: low behind, the aircraft pulling up and away",
        path: Path::Line { from: v(0.3, -0.05, 1.6), to: v(0.35, 0.0, 2.6) },
        look_from: v(0.0, 0.0, -0.1),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.4, 1.9),
        bank_follow: 0.2,
        trigger: Some(Trigger::Rotation),
        ..BASE
    },
    Shot {
        id: "roll_ahead",
        name: "Takeoff roll: runway edge",
        group: Group::Action,
        blurb: "Takeoff roll: long lens from the runway edge ahead, the aircraft charging past",
        path: Path::Flyby { side: 0.9, height: 0.0, on_terrain: true },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (3.0, 1.4),
        bank_follow: 0.0,
        trigger: Some(Trigger::TakeoffRoll),
        ..BASE
    },
    Shot {
        id: "roll_behind",
        name: "Takeoff roll: from behind",
        group: Group::Action,
        blurb: "Takeoff roll: low and far behind on a long lens",
        path: Path::Line { from: v(0.35, -0.02, 3.5), to: v(0.4, 0.04, 5.0) },
        look_from: v(0.0, 0.0, -0.1),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (2.8, 3.4),
        bank_follow: 0.0,
        trigger: Some(Trigger::TakeoffRoll),
        ..BASE
    },
    Shot {
        id: "spoilers_window",
        name: "Spoilers: window seat",
        group: Group::Action,
        blurb: "Ground spoilers: from the cabin, looking out over the wing",
        path: Path::Mount { at: v(0.075, 0.04, 0.1), offset_m: v(0.35, 0.0, 0.0) },
        look_from: v(0.32, -0.01, 0.04),
        look_to: v(0.32, -0.01, 0.05),
        fov_deg: Some(75.0),
        bank_follow: 0.0,
        trigger: Some(Trigger::Spoilers),
        ..BASE
    },
    Shot {
        id: "spoilers_above",
        name: "Spoilers: above the wing",
        group: Group::Action,
        blurb: "Ground spoilers: from above and behind, down onto the wing",
        path: Path::Line { from: v(0.5, 0.45, 0.8), to: v(0.45, 0.4, 0.55) },
        look_from: v(0.3, -0.02, 0.03),
        look_to: v(0.3, -0.02, 0.03),
        zoom: (1.5, 1.7),
        trigger: Some(Trigger::Spoilers),
        ..BASE
    },
    Shot {
        id: "start_intake",
        name: "Engine start: intake",
        group: Group::Action,
        blurb: "Engine start: ahead of the engine that lit, fan or prop spinning up",
        path: Path::Line { from: v(1.0, -0.2, -3.2), to: v(0.8, -0.15, -2.6) },
        target: Target::Engine { outboard_only: false },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.1),
        zoom: (1.3, 1.5),
        dutch: 3.0,
        trigger: Some(Trigger::EngineStart),
        ..BASE
    },
    Shot {
        id: "start_exhaust",
        name: "Engine start: exhaust",
        group: Group::Action,
        blurb: "Engine start: behind the engine that lit, the exhaust coming to life",
        // Behind and outboard of the tailpipe, drifting in and across.
        path: Path::Line { from: v(2.4, 0.2, 3.4), to: v(1.6, 0.1, 2.8) },
        target: Target::Engine { outboard_only: false },
        look_from: v(0.0, 0.0, 0.7),
        look_to: v(0.0, 0.0, 0.5),
        zoom: (1.3, 1.4),
        dutch: -2.0,
        trigger: Some(Trigger::EngineStart),
        ..BASE
    },
    Shot {
        id: "turn_high",
        name: "Steep turn: high outside",
        group: Group::Action,
        blurb: "Steep turn: above the high wing, horizon level, the aircraft banked hard",
        // Played from the low wing's side, so -x is the high wing.
        path: Path::Line { from: v(-0.7, 0.9, 1.3), to: v(-0.5, 1.0, 0.8) },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.2, 1.25),
        bank_follow: 0.0,
        on_ground: false,
        min_agl: 2.0,
        trigger: Some(Trigger::SteepTurn),
        ..BASE
    },
    Shot {
        id: "turn_fin",
        name: "Steep turn: down the low wing",
        group: Group::Action,
        blurb: "Steep turn: rigid camera on the fin, looking down the low wing at the ground",
        path: Path::Mount { at: v(0.0, 0.22, 0.4), offset_m: v(0.0, 0.4, 0.0) },
        look_from: v(0.45, -0.03, -0.05),
        look_to: v(0.5, -0.06, -0.1),
        fov_deg: Some(90.0),
        bank_follow: 0.0,
        on_ground: false,
        trigger: Some(Trigger::SteepTurn),
        ..BASE
    },
    Shot {
        id: "lights_head_on",
        name: "Lights: head-on",
        group: Group::Action,
        blurb: "Landing lights on at dusk or night: ahead, looking into them",
        path: Path::Line { from: v(0.1, -0.05, -2.6), to: v(0.15, -0.02, -2.1) },
        look_from: v(0.0, 0.0, -0.3),
        look_to: v(0.0, 0.0, -0.3),
        zoom: (1.4, 1.5),
        bank_follow: 0.5,
        trigger: Some(Trigger::Lights),
        ..BASE
    },
    Shot {
        id: "lights_front_quarter",
        name: "Lights: front quarter",
        group: Group::Action,
        blurb: "Landing lights on at dusk or night: low off the nose",
        path: Path::Line { from: v(0.7, -0.08, -1.5), to: v(0.9, -0.03, -1.2) },
        look_from: v(0.0, 0.0, -0.2),
        look_to: v(0.0, 0.0, -0.2),
        zoom: (1.3, 1.5),
        dutch: 4.0,
        trigger: Some(Trigger::Lights),
        ..BASE
    },
    Shot {
        id: "doors",
        name: "Doors: cabin door",
        group: Group::Action,
        blurb: "Door or canopy opening: beside the forward fuselage, left side",
        // The main door is on the left, forward, on almost every type.
        path: Path::Line { from: v(-0.9, 0.05, -0.55), to: v(-0.8, 0.08, -0.4) },
        look_from: v(-0.03, 0.0, -0.3),
        look_to: v(-0.03, 0.0, -0.28),
        zoom: (1.6, 1.7),
        mirror: false,
        on_ground: true,
        in_air: false,
        trigger: Some(Trigger::Doors),
        ..BASE
    },
    Shot {
        id: "pushback",
        name: "Pushback: from the ramp",
        group: Group::Action,
        blurb: "Pushback: high ahead and to the side, the aircraft rolling back",
        path: Path::Line { from: v(0.9, 0.6, -1.6), to: v(1.0, 0.65, -1.4) },
        look_from: v(0.0, 0.0, -0.1),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (1.1, 1.15),
        bank_follow: 0.0,
        in_air: false,
        trigger: Some(Trigger::Pushback),
        ..BASE
    },
    Shot {
        id: "shutdown",
        name: "Shutdown: pull back",
        group: Group::Action,
        blurb: "Engines off: a long, slow pull back and up to end the flight",
        path: Path::Line { from: v(1.0, 0.15, -1.0), to: v(2.4, 1.0, -2.4) },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.2, 1.0),
        bank_follow: 0.0,
        in_air: false,
        trigger: Some(Trigger::Shutdown),
        ..BASE
    },
    Shot {
        id: "goaround_climb",
        name: "Go-around: climb-out",
        group: Group::Action,
        blurb: "Go-around: low behind on a long lens as it powers away",
        path: Path::Line { from: v(0.3, -0.1, 3.5), to: v(0.35, 0.05, 4.5) },
        look_from: v(0.0, 0.0, -0.1),
        look_to: v(0.0, 0.0, -0.1),
        zoom: (2.5, 3.0),
        bank_follow: 0.3,
        on_ground: false,
        trigger: Some(Trigger::GoAround),
        ..BASE
    },
    Shot {
        id: "goaround_under",
        name: "Go-around: under the climb",
        group: Group::Action,
        blurb: "Go-around: standing under the flight path as it climbs over",
        path: Path::Flyby { side: 0.3, height: 0.0, on_terrain: true },
        look_from: v(0.0, 0.0, 0.0),
        look_to: v(0.0, 0.0, 0.0),
        zoom: (1.0, 1.0),
        bank_follow: 0.0,
        on_ground: false,
        trigger: Some(Trigger::GoAround),
        ..BASE
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_unique_and_prefs_safe() {
        for (i, a) in CATALOG.iter().enumerate() {
            assert!(a.id.chars().all(|c| c.is_ascii_lowercase() || c == '_'), "{}", a.id);
            assert!(CATALOG[i + 1..].iter().all(|b| b.id != a.id), "duplicate {}", a.id);
        }
    }

    #[test]
    fn some_shot_works_anywhere() {
        assert!(CATALOG
            .iter()
            .any(|s| s.in_air && s.on_ground && s.min_agl == 0.0 && s.min_speed == 0.0 && s.trigger.is_none()));
    }

    #[test]
    fn action_shots_are_exactly_the_triggered_ones() {
        for s in CATALOG {
            assert_eq!(s.group == Group::Action, s.trigger.is_some(), "{}", s.id);
        }
    }
}
