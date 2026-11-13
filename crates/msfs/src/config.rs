//! `camship-msfs.ini`: how to reach the simulator and how its camera API
//! reads the numbers. Written with defaults on first run, never overwritten.

use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PositionMode {
    /// Camera position as an offset from the aircraft in its body axes.
    Aircraft,
    /// Camera position as latitude, longitude and altitude.
    World,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    /// `host:port` for a TCP connection; empty for the local named pipe.
    pub address: String,
    /// Your normal horizontal field of view, degrees (zoom 1).
    pub fov_deg: f64,
    /// Whether the camera API takes the field of view in radians.
    pub fov_radians: bool,
    pub position: PositionMode,
    /// Multiplies the camera pitch sent to MSFS (-1: positive is nose down).
    pub pitch_sign: f64,
    /// Multiplies the camera bank sent to MSFS (-1: positive is left).
    pub bank_sign: f64,
    /// Camera to switch to when Camship lets go; empty for the simulator's choice.
    pub release_camera: String,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            address: String::new(),
            fov_deg: 60.0,
            fov_radians: true,
            position: PositionMode::Aircraft,
            pitch_sign: -1.0,
            bank_sign: -1.0,
            release_camera: String::new(),
        }
    }
}

const TEMPLATE: &str = "\
# Camship for MSFS 2024: connection and camera settings.
# Shot and camera preferences live in camship.prf, next to this file.

# Leave empty to connect to MSFS on this PC (named pipe). Set host:port to
# use TCP instead, e.g. a sim on another PC with an IPv4 entry in SimConnect.xml.
address

# Your normal horizontal field of view, degrees. Zoomed shots narrow it.
fov_deg 60

# The rest calibrates how the camera API reads the numbers. Change these only
# if the camera points the wrong way.
#   fov_units      radians | degrees
#   position       aircraft (offset from the aircraft) | world (lat/lon/alt)
#   pitch_sign     -1 if MSFS camera pitch is positive nose down, else 1
#   bank_sign      -1 if MSFS camera bank is positive to the left, else 1
fov_units radians
position aircraft
pitch_sign -1
bank_sign -1

# Camera to return to when Camship stops (empty: MSFS decides).
release_camera
";

impl Config {
    pub fn parse(text: &str) -> Self {
        let mut c = Self::default();
        for line in text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
            let (key, value) = line.split_once(char::is_whitespace).map_or((line, ""), |(k, v)| (k, v.trim()));
            let num = || value.parse::<f64>().ok().filter(|v| v.is_finite());
            let sign = || num().map(|v| if v < 0.0 { -1.0 } else { 1.0 });
            match key {
                "address" => c.address = value.to_string(),
                "fov_deg" => c.fov_deg = num().map_or(c.fov_deg, |v| v.clamp(10.0, 150.0)),
                "fov_units" => c.fov_radians = !value.eq_ignore_ascii_case("degrees"),
                "position" => {
                    c.position = if value.eq_ignore_ascii_case("world") { PositionMode::World } else { PositionMode::Aircraft }
                }
                "pitch_sign" => c.pitch_sign = sign().unwrap_or(c.pitch_sign),
                "bank_sign" => c.bank_sign = sign().unwrap_or(c.bank_sign),
                "release_camera" => c.release_camera = value.to_string(),
                _ => {}
            }
        }
        c
    }

    /// Reads the file, writing the commented defaults first if it's missing.
    pub fn load(path: &Path) -> Self {
        match std::fs::read_to_string(path) {
            Ok(text) => Self::parse(&text),
            Err(_) => {
                let _ = std::fs::write(path, TEMPLATE);
                Self::parse(TEMPLATE)
            }
        }
    }
}

/// `%APPDATA%\Camship` on Windows, `~/.config/camship` elsewhere.
pub fn dir() -> PathBuf {
    let base = std::env::var_os("APPDATA")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .unwrap_or_else(|| PathBuf::from("."));
    let name = if cfg!(windows) { "Camship" } else { "camship" };
    base.join(name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn template_parses_to_defaults() {
        assert_eq!(Config::parse(TEMPLATE), Config::default());
    }

    #[test]
    fn overrides() {
        let c = Config::parse("address 192.168.1.5:500\nfov_units degrees\nposition world\npitch_sign 1\nfov_deg 400");
        assert_eq!(c.address, "192.168.1.5:500");
        assert!(!c.fov_radians);
        assert_eq!(c.position, PositionMode::World);
        assert_eq!(c.pitch_sign, 1.0);
        assert_eq!(c.fov_deg, 150.0);
    }
}
