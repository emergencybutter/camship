//! Camship for Microsoft Flight Simulator 2024: a small Windows app that
//! connects to the simulator over SimConnect and drives its camera with the
//! same director as the X-Plane plugin.

#![cfg_attr(all(windows, not(debug_assertions)), windows_subsystem = "windows")]

mod config;
mod geo;
mod sim;
mod simconnect;
mod worker;

// The settings window is the X-Plane plugin's, unchanged.
#[path = "../../plugin/src/view.rs"]
mod view;

use std::fs;
use std::path::PathBuf;
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use camship_core::{Director, Settings};

use config::Config;
use view::{Action, ViewModel};
use worker::{CameraState, Command, Shared};

/// Settings edited in the window are written to disk once they settle.
const SAVE_DELAY: Duration = Duration::from_millis(800);

fn main() -> eframe::Result {
    let dir = config::dir();
    let _ = fs::create_dir_all(&dir);
    let mut cfg = Config::load(&dir.join("camship-msfs.ini"));
    // `--address host:port` overrides the file, handy for a sim on another PC.
    let args: Vec<String> = std::env::args().collect();
    if let Some(i) = args.iter().position(|a| a == "--address") {
        if let Some(a) = args.get(i + 1) {
            cfg.address = a.clone();
        }
    }

    let prefs_path = dir.join("camship.prf");
    let settings = fs::read_to_string(&prefs_path).map_or_else(|_| Settings::default(), |t| Settings::parse(&t));
    let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(1, |d| d.as_nanos() as u64);
    let shared = Arc::new(Mutex::new(Shared::new(Director::new(settings, seed), dir.join("Log.txt"))));
    shared.lock().unwrap().log(&format!("Camship {} for MSFS 2024; settings in {}", env!("CARGO_PKG_VERSION"), dir.display()));

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Camship for MSFS 2024")
            .with_inner_size([470.0, 820.0])
            .with_min_inner_size([380.0, 400.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Camship for MSFS 2024",
        options,
        Box::new(move |cc| {
            view::install_style(&cc.egui_ctx);
            let (tx, rx) = mpsc::channel();
            let ctx = cc.egui_ctx.clone();
            let worker_shared = shared.clone();
            let worker = std::thread::Builder::new()
                .name("simconnect".into())
                .spawn(move || worker::run(worker_shared, rx, cfg, move || ctx.request_repaint()))?;
            Ok(Box::new(Gui { shared, tx, prefs_path, unsaved_since: None, worker: Some(worker), show_log: false }))
        }),
    )
}

struct Gui {
    shared: Arc<Mutex<Shared>>,
    tx: Sender<Command>,
    prefs_path: PathBuf,
    unsaved_since: Option<Instant>,
    worker: Option<std::thread::JoinHandle<()>>,
    show_log: bool,
}

impl Gui {
    fn save_prefs(&self, s: &Shared) {
        let _ = fs::write(&self.prefs_path, s.director.settings().to_text());
    }
}

impl eframe::App for Gui {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        ui.ctx().request_repaint_after(Duration::from_millis(250));
        let shared = self.shared.clone();
        let mut s = shared.lock().unwrap_or_else(|e| e.into_inner());

        let mut actions = Vec::new();
        let before = s.director.settings().clone();
        let mut settings = before.clone();
        let shots = s.director.shots();
        let supported: Vec<bool> = (0..shots.len()).map(|i| s.director.supported(i, s.size)).collect();

        ui.painter().rect_filled(ui.max_rect(), 0.0, ui.visuals().panel_fill);
        egui::Frame::new().inner_margin(egui::Margin { left: 12, right: 12, top: 8, bottom: 0 }).show(ui, |ui| {
            ui.horizontal(|ui| {
                let (dot, text) = match (s.connected, s.camera) {
                    (false, _) => (egui::Color32::from_rgb(140, 148, 160), s.status.clone()),
                    (true, CameraState::Off) => (egui::Color32::from_rgb(70, 190, 110), "Connected to MSFS".into()),
                    (true, CameraState::Acquiring) => (egui::Color32::from_rgb(240, 170, 40), "Taking the camera...".into()),
                    (true, CameraState::Running) => (egui::Color32::from_rgb(64, 140, 235), "Filming".into()),
                };
                ui.label(egui::RichText::new("●").color(dot));
                ui.label(text);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button(if self.show_log { "Hide log" } else { "Log" }).clicked() {
                        self.show_log = !self.show_log;
                    }
                });
            });
            if self.show_log {
                egui::ScrollArea::vertical().max_height(140.0).stick_to_bottom(true).show(ui, |ui| {
                    for line in &s.log {
                        ui.label(egui::RichText::new(line).monospace().small());
                    }
                });
            }
        });

        let aircraft = s.aircraft.clone();
        let mut vm = ViewModel {
            settings: &mut settings,
            shots,
            supported: &supported,
            running: s.camera != CameraState::Off,
            current: s.director.current_shot(),
            hold: s.director.hold(),
            aircraft: &aircraft,
        };
        view::show(ui, &mut vm, true, &mut actions);

        if settings != before {
            *s.director.settings_mut() = settings;
            self.unsaved_since = Some(Instant::now());
        }
        if self.unsaved_since.is_some_and(|t| t.elapsed() >= SAVE_DELAY) {
            self.unsaved_since = None;
            self.save_prefs(&s);
        }
        drop(s);

        for a in actions {
            let _ = self.tx.send(match a {
                Action::Toggle => Command::Toggle,
                Action::Next => Command::Next,
                Action::SetHold(h) => Command::SetHold(h),
                Action::Play(i) => Command::Play(i),
            });
        }
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        let _ = self.tx.send(Command::Quit);
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
        let s = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        self.save_prefs(&s);
    }
}
