//! The FlyCam window's content, as plain egui code.

use egui::{vec2, Align, Button, Color32, CornerRadius, DragValue, Frame, Layout, Margin, RichText, Sense, Slider, Stroke, Ui};
use flycam_core::{Group, Settings, Shot};

const BG: Color32 = Color32::from_rgb(22, 25, 30);
const CARD: Color32 = Color32::from_rgb(31, 35, 42);
const BORDER: Color32 = Color32::from_rgb(45, 50, 59);
const TEXT: Color32 = Color32::from_rgb(226, 230, 236);
const MUTED: Color32 = Color32::from_rgb(140, 148, 160);
const ACCENT: Color32 = Color32::from_rgb(64, 140, 235);
const GREEN: Color32 = Color32::from_rgb(70, 190, 110);
const AMBER: Color32 = Color32::from_rgb(240, 170, 40);
const RED: Color32 = Color32::from_rgb(225, 85, 75);

pub struct ViewModel<'a> {
    pub settings: &'a mut Settings,
    pub shots: &'static [Shot],
    /// Per shot: whether this aircraft has what it needs (e.g. engine positions).
    pub supported: &'a [bool],
    pub running: bool,
    pub current: Option<&'static Shot>,
    pub hold: bool,
    /// One line about the aircraft, e.g. "34 m · 2 engines".
    pub aircraft: &'a str,
}

pub enum Action {
    Toggle,
    Next,
    SetHold(bool),
    Play(usize),
}

pub fn install_style(ctx: &egui::Context) {
    let mut v = egui::Visuals::dark();
    v.panel_fill = BG;
    v.window_fill = BG;
    v.extreme_bg_color = Color32::from_rgb(16, 18, 22);
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = ACCENT.linear_multiply(0.6);
    v.selection.stroke = Stroke::new(1.0, ACCENT);
    let radius = CornerRadius::same(5);
    for w in [
        &mut v.widgets.noninteractive,
        &mut v.widgets.inactive,
        &mut v.widgets.hovered,
        &mut v.widgets.active,
        &mut v.widgets.open,
    ] {
        w.corner_radius = radius;
    }
    v.widgets.inactive.weak_bg_fill = Color32::from_rgb(42, 47, 56);
    v.widgets.inactive.bg_fill = Color32::from_rgb(42, 47, 56);
    v.widgets.hovered.weak_bg_fill = Color32::from_rgb(52, 58, 69);
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT.linear_multiply(0.7));
    v.widgets.active.bg_stroke = Stroke::new(1.0, ACCENT);
    ctx.set_visuals(v);
    ctx.style_mut_of(egui::Theme::Dark, |s| {
        s.spacing.item_spacing = vec2(8.0, 6.0);
        s.spacing.button_padding = vec2(8.0, 4.0);
        s.spacing.interact_size.y = 22.0;
        use egui::{FontFamily::Proportional, FontId, TextStyle};
        s.text_styles.insert(TextStyle::Body, FontId::new(14.0, Proportional));
        s.text_styles.insert(TextStyle::Button, FontId::new(14.0, Proportional));
        s.text_styles.insert(TextStyle::Small, FontId::new(11.5, Proportional));
        s.text_styles.insert(TextStyle::Heading, FontId::new(17.0, Proportional));
    });
    ctx.set_theme(egui::Theme::Dark);
}

/// Draws the window and returns the height its content needs, in points.
///
/// The shot list is long, so it always scrolls inside a capped height; a
/// popped-out window scrolls as a whole.
pub fn show(ui: &mut Ui, vm: &mut ViewModel, popped_out: bool, actions: &mut Vec<Action>) -> f32 {
    ui.painter().rect_filled(ui.max_rect(), 0.0, BG);
    let mut content = |ui: &mut Ui| {
        Frame::new()
            .inner_margin(Margin::same(12))
            .show(ui, |ui| {
                header(ui, vm, actions);
                ui.add_space(10.0);
                shots(ui, vm, actions, !popped_out);
                ui.add_space(8.0);
                camera(ui, vm.settings);
                footer(ui, vm.aircraft);
            })
            .response
            .rect
            .height()
    };
    if popped_out {
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, content).inner
    } else {
        content(ui)
    }
}

fn card(ui: &mut Ui, add: impl FnOnce(&mut Ui)) {
    Frame::new()
        .fill(CARD)
        .stroke(Stroke::new(1.0, BORDER))
        .corner_radius(6)
        .inner_margin(Margin::same(10))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            add(ui)
        });
}

fn header(ui: &mut Ui, vm: &ViewModel, actions: &mut Vec<Action>) {
    let (color, label) = match (vm.running, vm.hold) {
        (true, true) => (AMBER, "FILMING · held"),
        (true, false) => (GREEN, "FILMING"),
        (false, _) => (MUTED, "OFF"),
    };
    ui.horizontal(|ui| {
        let (dot, _) = ui.allocate_exact_size(vec2(12.0, 12.0), Sense::hover());
        ui.painter().circle_filled(dot.center(), 5.0, color);
        ui.label(RichText::new(label).strong().size(16.0).color(color));
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            let (text, fill) = if vm.running { ("Stop", RED) } else { ("Start", ACCENT) };
            let button = Button::new(RichText::new(text).strong().size(15.0).color(Color32::WHITE))
                .fill(fill)
                .min_size(vec2(80.0, 28.0));
            if ui.add(button).clicked() {
                actions.push(Action::Toggle);
            }
        });
    });
    ui.horizontal(|ui| {
        let shot = match (vm.running, vm.current) {
            (true, Some(s)) => RichText::new(s.name).color(TEXT),
            _ => RichText::new("Cinematic camera is off").color(MUTED),
        };
        ui.label(shot);
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            let mut hold = vm.hold;
            if ui.add_enabled(vm.running, egui::Checkbox::new(&mut hold, "Hold")).on_hover_text("Keep the current shot going").changed() {
                actions.push(Action::SetHold(hold));
            }
            if ui.add_enabled(vm.running, Button::new("Next shot")).clicked() {
                actions.push(Action::Next);
            }
        });
    });
}

fn shots(ui: &mut Ui, vm: &mut ViewModel, actions: &mut Vec<Action>, cap_height: bool) {
    card(ui, |ui| {
        let on = vm.shots.iter().filter(|s| !vm.settings.is_disabled(s.id)).count();
        ui.horizontal(|ui| {
            ui.label(RichText::new("Shots").strong().size(15.0));
            ui.label(RichText::new(format!("{on} of {} on", vm.shots.len())).small().color(MUTED));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui.small_button("None").clicked() {
                    vm.shots.iter().for_each(|s| vm.settings.set_disabled(s.id, true));
                }
                if ui.small_button("All").clicked() {
                    vm.settings.disabled_shots.clear();
                }
            });
        });
        if on == 0 {
            ui.label(RichText::new("No shot ticked: FlyCam will use all of them.").small().color(AMBER));
        }
        let mut list = |ui: &mut Ui| {
            for group in Group::ALL {
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.label(RichText::new(group.title()).color(ACCENT).strong());
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        let members = vm.shots.iter().filter(|s| s.group == group);
                        if ui.small_button("None").clicked() {
                            members.clone().for_each(|s| vm.settings.set_disabled(s.id, true));
                        }
                        if ui.small_button("All").clicked() {
                            members.for_each(|s| vm.settings.set_disabled(s.id, false));
                        }
                    });
                });
                for (i, shot) in vm.shots.iter().enumerate().filter(|(_, s)| s.group == group) {
                    shot_row(ui, vm, i, shot, actions);
                }
            }
        };
        if cap_height {
            egui::ScrollArea::vertical().max_height(330.0).auto_shrink([false, true]).show(ui, list);
        } else {
            list(ui);
        }
    });
}

fn shot_row(ui: &mut Ui, vm: &mut ViewModel, i: usize, shot: &Shot, actions: &mut Vec<Action>) {
    let supported = vm.supported.get(i).copied().unwrap_or(true);
    let playing = vm.running && vm.current.is_some_and(|c| c.id == shot.id);
    ui.horizontal(|ui| {
        let mut enabled = !vm.settings.is_disabled(shot.id);
        let name = if playing {
            RichText::new(shot.name).color(GREEN).strong()
        } else if supported {
            RichText::new(shot.name)
        } else {
            RichText::new(shot.name).color(MUTED)
        };
        if ui.checkbox(&mut enabled, name).changed() {
            vm.settings.set_disabled(shot.id, !enabled);
        }
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            let can_play = vm.running && supported;
            let play = ui
                .add_enabled(can_play, Button::new("").small().min_size(vec2(22.0, 18.0)))
                .on_hover_text("Cut to this shot now")
                .on_disabled_hover_text(if supported { "Start the camera first" } else { "Not available for this aircraft" });
            // Painted, not a glyph: egui's built-in fonts have no play symbol.
            let c = play.rect.center();
            let color = if can_play { TEXT } else { MUTED.linear_multiply(0.6) };
            let tri = vec![c + vec2(-3.5, -5.0), c + vec2(-3.5, 5.0), c + vec2(5.0, 0.0)];
            ui.painter().add(egui::Shape::convex_polygon(tri, color, Stroke::NONE));
            if play.clicked() {
                actions.push(Action::Play(i));
            }
            let blurb = if supported { shot.blurb } else { "no engine data for this aircraft" };
            ui.add(egui::Label::new(RichText::new(blurb).small().color(MUTED)).truncate());
        });
    });
}

fn camera(ui: &mut Ui, s: &mut Settings) {
    card(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Camera").strong().size(15.0));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui.small_button("Defaults").on_hover_text("Reset these settings (keeps your shot list)").clicked() {
                    let shots = std::mem::take(&mut s.disabled_shots);
                    *s = Settings { disabled_shots: shots, ..Settings::default() };
                }
            });
        });
        egui::Grid::new("camera").num_columns(2).spacing(vec2(12.0, 8.0)).show(ui, |ui| {
            ui.label("Shot length");
            ui.horizontal(|ui| {
                ui.add(DragValue::new(&mut s.shot_min_s).range(2.0..=60.0).speed(0.1).suffix(" s").max_decimals(1));
                ui.label("to");
                ui.add(DragValue::new(&mut s.shot_max_s).range(2.0..=60.0).speed(0.1).suffix(" s").max_decimals(1));
            });
            ui.end_row();
            if s.shot_max_s < s.shot_min_s {
                s.shot_max_s = s.shot_min_s;
            }

            ui.label("Distance").on_hover_text("How far the cameras sit from the aircraft");
            ui.add(Slider::new(&mut s.distance_scale, 0.5..=3.0).suffix("×").max_decimals(2));
            ui.end_row();

            ui.label("Zoom");
            ui.add(Slider::new(&mut s.zoom_scale, 0.5..=2.0).suffix("×").max_decimals(2));
            ui.end_row();

            ui.label("Turn lag").on_hover_text("How late the camera plane follows your turns. 0 locks the aircraft in the frame");
            ui.add(Slider::new(&mut s.formation_lag_s, 0.0..=3.0).suffix(" s").max_decimals(1));
            ui.end_row();

            ui.label("Between shots");
            ui.horizontal(|ui| {
                ui.radio_value(&mut s.glide, false, "Cut");
                ui.radio_value(&mut s.glide, true, "Glide");
                if s.glide {
                    ui.add(DragValue::new(&mut s.glide_s).range(0.2..=10.0).speed(0.05).suffix(" s").max_decimals(1));
                }
            });
            ui.end_row();
        });
    });
}

fn footer(ui: &mut Ui, aircraft: &str) {
    ui.add_space(6.0);
    ui.separator();
    ui.label(RichText::new(aircraft).small().color(MUTED));
    ui.label(
        RichText::new(concat!(
            "FlyCam v",
            env!("CARGO_PKG_VERSION"),
            " · Keys: flycam/toggle, flycam/next_shot, flycam/hold, flycam/toggle_window"
        ))
        .small()
        .color(MUTED),
    );
}
