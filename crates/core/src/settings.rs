//! User settings and the `camship.prf` text format (`key value` per line).

#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    /// Each take lasts a random time between these, in seconds.
    pub shot_min_s: f64,
    pub shot_max_s: f64,
    /// Glide the camera between shots instead of cutting.
    pub glide: bool,
    pub glide_s: f64,
    /// Multiplies how far every camera sits from the aircraft.
    pub distance_scale: f64,
    /// How far behind the subject's manoeuvres the camera ship lags, seconds.
    pub formation_lag_s: f64,
    /// How much the camera ship's formation position follows the subject's bank (0..1).
    pub frame_roll: f64,
    /// Minimum camera height above terrain, meters.
    pub ground_clearance_m: f64,
    /// Subject size in meters; 0 means use the aircraft's own dimensions.
    pub subject_size_m: f64,
    /// Multiplies every shot's zoom (use < 1 if your default FOV is narrow).
    pub zoom_scale: f64,
    /// Shot ids the director must never pick.
    pub disabled_shots: Vec<String>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            shot_min_s: 6.0,
            shot_max_s: 10.0,
            glide: false,
            glide_s: 2.0,
            distance_scale: 1.5,
            formation_lag_s: 1.2,
            frame_roll: 0.5,
            ground_clearance_m: 2.0,
            subject_size_m: 0.0,
            zoom_scale: 1.0,
            disabled_shots: Vec::new(),
        }
    }
}

impl Settings {
    pub fn effective_glide_s(&self) -> f64 {
        if self.glide {
            self.glide_s
        } else {
            0.0
        }
    }

    pub fn is_disabled(&self, id: &str) -> bool {
        self.disabled_shots.iter().any(|d| d == id)
    }

    pub fn set_disabled(&mut self, id: &str, disabled: bool) {
        self.disabled_shots.retain(|d| d != id);
        if disabled {
            self.disabled_shots.push(id.to_string());
        }
    }

    /// Parses a prefs file. Unknown keys and bad values are ignored, so an
    /// old or hand-edited file never stops the plugin from loading.
    pub fn parse(text: &str) -> Self {
        let mut s = Self::default();
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let (key, value) = match line.split_once(char::is_whitespace) {
                Some((k, v)) => (k, v.trim()),
                None => (line, ""),
            };
            let num = |lo: f64, hi: f64| value.parse::<f64>().ok().filter(|v| v.is_finite()).map(|v| v.clamp(lo, hi));
            let flag = || match value {
                "1" | "true" | "on" | "yes" => Some(true),
                "0" | "false" | "off" | "no" => Some(false),
                _ => None,
            };
            macro_rules! set {
                ($field:ident, $v:expr) => {
                    if let Some(v) = $v {
                        s.$field = v;
                    }
                };
            }
            match key {
                "shot_min_s" => set!(shot_min_s, num(2.0, 60.0)),
                "shot_max_s" => set!(shot_max_s, num(2.0, 60.0)),
                "glide" => set!(glide, flag()),
                "glide_s" => set!(glide_s, num(0.2, 10.0)),
                "distance_scale" => set!(distance_scale, num(0.5, 5.0)),
                "formation_lag_s" => set!(formation_lag_s, num(0.0, 10.0)),
                "frame_roll" => set!(frame_roll, num(0.0, 1.0)),
                "ground_clearance_m" => set!(ground_clearance_m, num(0.5, 100.0)),
                "subject_size_m" => set!(subject_size_m, num(0.0, 200.0)),
                "zoom_scale" => set!(zoom_scale, num(0.2, 5.0)),
                "disabled_shots" => {
                    s.disabled_shots = value.split(',').map(str::trim).filter(|v| !v.is_empty()).map(String::from).collect()
                }
                _ => {}
            }
        }
        if s.shot_max_s < s.shot_min_s {
            s.shot_max_s = s.shot_min_s;
        }
        s
    }

    pub fn to_text(&self) -> String {
        let b = |v: bool| if v { "1" } else { "0" };
        format!(
            "# Camship preferences. Edit while X-Plane is not running.\n\
             shot_min_s {}\nshot_max_s {}\nglide {}\nglide_s {}\ndistance_scale {}\n\
             formation_lag_s {}\nframe_roll {}\nground_clearance_m {}\nsubject_size_m {}\n\
             zoom_scale {}\ndisabled_shots {}\n",
            self.shot_min_s,
            self.shot_max_s,
            b(self.glide),
            self.glide_s,
            self.distance_scale,
            self.formation_lag_s,
            self.frame_roll,
            self.ground_clearance_m,
            self.subject_size_m,
            self.zoom_scale,
            self.disabled_shots.join(","),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let mut s = Settings { glide: true, distance_scale: 2.5, ..Settings::default() };
        s.set_disabled("belly", true);
        s.set_disabled("flyby", true);
        assert_eq!(Settings::parse(&s.to_text()), s);
    }

    #[test]
    fn bad_values_fall_back_and_ranges_clamp() {
        let s = Settings::parse("shot_min_s banana\ndistance_scale 99\nbogus 1\nshot_max_s 3\nglide maybe\n");
        assert_eq!(s.distance_scale, 5.0);
        assert_eq!(s.shot_min_s, 6.0);
        // max below min is lifted to min.
        assert_eq!(s.shot_max_s, 6.0);
        assert!(!s.glide);
    }
}
