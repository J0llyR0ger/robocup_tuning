use std::f32::consts::PI;

use raylib::prelude::*;
use zerocopy::IntoBytes;

use crate::{
    data_source::CommandSink,
    protocol::{
        COMMAND_HEADER, Cluster, CommandPacket, KEY_DRIVE_ERROR, KEY_HEADING, KEY_LEFT_COMMAND,
        KEY_LEFT_WHEEL_VELOCITY, KEY_LOOKAHEAD_X, KEY_LOOKAHEAD_Y, KEY_NEXTPOINT_X,
        KEY_NEXTPOINT_Y, KEY_PITCH, KEY_POSITION_UNCERTAINTY, KEY_POSITION_X, KEY_POSITION_Y,
        KEY_RIGHT_COMMAND, KEY_RIGHT_WHEEL_VELOCITY, KEY_TURN_ERROR, KEY_WEIGHT_ASPECT,
        KEY_WEIGHT_CONFIDENCE, KEY_WEIGHT_DIAMETER, KEY_WEIGHT_EXTENT, KEY_WEIGHT_POINT_COUNT,
        KEY_WEIGHT_RANGE, KEY_WEIGHT_SPREAD, KEY_WEIGHT_TARGET_X, KEY_WEIGHT_TARGET_Y,
        VALUE_TYPE_FLOAT32,
    },
    telemetry_state::{TELEMETRY, TypedValue},
};

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 720;
const FIELD_WIDTH_X_METERS: f32 = 2.425;
const FIELD_HEIGHT_Y_METERS: f32 = 4.85;
const VIEW_MARGIN_PX: f32 = 60.0;

const SLIDER_WIDTH_PX: f32 = 220.0;
const SLIDER_HEIGHT_PX: f32 = 14.0;
const SLIDER_GAP_PX: f32 = 26.0;

#[derive(Copy, Clone, Eq, PartialEq)]
enum CenterMode {
    RobotOffset,
    MapCenter,
}

#[derive(Copy, Clone, Default)]
struct RunningStats {
    samples: u32,
    mean: f64,
    m2: f64,
}

impl RunningStats {
    fn update(&mut self, value: f32) {
        self.samples += 1;

        let value = value as f64;
        let delta = value - self.mean;
        self.mean += delta / self.samples as f64;
        let delta2 = value - self.mean;
        self.m2 += delta * delta2;
    }

    fn mean(&self) -> f32 {
        self.mean as f32
    }

    fn stddev(&self) -> f32 {
        if self.samples <= 1 {
            0.0
        } else {
            (self.m2 / (self.samples as f64 - 1.0)).sqrt() as f32
        }
    }
}

#[derive(Copy, Clone, Default)]
struct ClusterStatsSummary {
    spread: RunningStats,
    extent: RunningStats,
    diameter_mm: RunningStats,
    aspect_ratio: RunningStats,
    range: RunningStats,
    point_count: RunningStats,
}

impl ClusterStatsSummary {
    fn update(&mut self, cluster: Cluster) {
        self.spread.update(cluster.spread);
        self.extent.update(cluster.max_extent);
        self.diameter_mm.update(cluster.diameter_mm);
        self.aspect_ratio.update(cluster.aspect_ratio);
        self.range.update(cluster.range);
        self.point_count.update(cluster.count as f32);
    }
}

#[derive(Copy, Clone)]
struct TrackingCircle {
    center_world: Vector2,
    radius_m: f32,
    matched_frames: u32,
    missed_frames: u32,
    stats: ClusterStatsSummary,
}

impl TrackingCircle {
    fn new(center_world: Vector2, radius_m: f32) -> Self {
        Self {
            center_world,
            radius_m,
            matched_frames: 0,
            missed_frames: 0,
            stats: ClusterStatsSummary::default(),
        }
    }

    fn observe_cluster(&mut self, cluster: Cluster) {
        self.matched_frames += 1;
        self.stats.update(cluster);
    }
}

#[derive(Copy, Clone)]
struct HoverCluster {
    cluster: Cluster,
    radius_m: f32,
}

fn point_in_rect(point: Vector2, rect: Rectangle) -> bool {
    point.x >= rect.x
        && point.x <= rect.x + rect.width
        && point.y >= rect.y
        && point.y <= rect.y + rect.height
}

fn draw_slider(
    d: &mut RaylibDrawHandle,
    label: &str,
    rect: Rectangle,
    value: &mut f32,
    min_value: f32,
    max_value: f32,
) {
    let normalized = ((*value - min_value) / (max_value - min_value)).clamp(0.0, 1.0);
    let knob_x = rect.x + normalized * rect.width;

    d.draw_text(
        format!("{label}: {:.2}", *value).as_str(),
        rect.x as i32,
        (rect.y - 20.0) as i32,
        18,
        Color::BLACK,
    );
    d.draw_rectangle_rec(rect, Color::LIGHTGRAY);
    d.draw_rectangle_lines_ex(rect, 1.0, Color::GRAY);
    d.draw_circle_v(
        Vector2::new(knob_x, rect.y + rect.height * 0.5),
        rect.height * 0.6,
        Color::DARKBLUE,
    );
}

fn update_slider_value(
    rect: Rectangle,
    value: &mut f32,
    min_value: f32,
    max_value: f32,
    mouse_pos: Vector2,
    mouse_down: bool,
) {
    if mouse_down && point_in_rect(mouse_pos, rect) {
        let t = ((mouse_pos.x - rect.x) / rect.width).clamp(0.0, 1.0);
        *value = min_value + t * (max_value - min_value);
    }
}

fn value_as_f32(value: Option<&TypedValue>) -> Option<f32> {
    match value {
        Some(v) if v.value_type == VALUE_TYPE_FLOAT32 => {
            let value = f32::from_bits(v.payload);
            value.is_finite().then_some(value)
        }
        _ => None,
    }
}

pub fn run_ui(command_sink: &mut CommandSink) -> Result<(), Box<dyn std::error::Error>> {
    let (mut rl, thread) = raylib::init()
        .size(WIDTH, HEIGHT)
        .title("Hello, World")
        .build();

    rl.set_target_fps(30);

    let mut map_scale = 1.0f32;
    let mut center_mode = CenterMode::RobotOffset;
    let mut robot_offset_x = 0.0f32;
    let mut robot_offset_y = 0.0f32;
    let mut map_center_x_slider = FIELD_WIDTH_X_METERS * 0.5;
    let mut map_center_y_slider = FIELD_HEIGHT_Y_METERS * 0.5;
    let mut prev_mouse_down = false;
    let mut tracking_circle: Option<TrackingCircle> = None;
    let mut target_cycle_index: usize = 0;
    let mut summing_paused = false;

    while !rl.window_should_close() {
        let mut y = 0;
        if rl.is_key_down(KeyboardKey::KEY_W) {
            y = 1;
        } else if rl.is_key_down(KeyboardKey::KEY_S) {
            y = -1;
        }

        let mut x = 0;
        if rl.is_key_down(KeyboardKey::KEY_A) {
            x = -1;
        } else if rl.is_key_down(KeyboardKey::KEY_D) {
            x = 1;
        }

        let packet = CommandPacket {
            left_command: (y + x).clamp(-1, 1) * 100,
            right_command: (y - x).clamp(-1, 1) * 100,
        };

        let mut payload = Vec::with_capacity(COMMAND_HEADER.len() + packet.as_bytes().len());
        payload.extend_from_slice(&COMMAND_HEADER);
        payload.extend_from_slice(packet.as_bytes());

        // Sending commands is causing pipe halts

        // match command_sink {
        //     CommandSink::Serial(sender) => {
        //         if sender.send(payload).is_err() {
        //             *command_sink = CommandSink::None;
        //         }
        //     }
        //     CommandSink::WebSocket(sender) => {
        //         if sender.send(payload).is_err() {
        //             *command_sink = CommandSink::None;
        //         }
        //     }
        //     CommandSink::None => {}
        // }

        let mouse_pos = rl.get_mouse_position();
        let mouse_down = rl.is_mouse_button_down(MouseButton::MOUSE_BUTTON_LEFT);
        let mouse_pressed = mouse_down && !prev_mouse_down;
        let right_mouse_pressed = rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_RIGHT);
        let toggle_pressed = rl.is_key_pressed(KeyboardKey::KEY_T);
        let clear_tracker_pressed = rl.is_key_pressed(KeyboardKey::KEY_BACKSPACE);
        let cycle_target_pressed = rl.is_key_pressed(KeyboardKey::KEY_TAB);
        let pause_summing_pressed = rl.is_key_pressed(KeyboardKey::KEY_SPACE);
        prev_mouse_down = mouse_down;

        if pause_summing_pressed {
            summing_paused = !summing_paused;
        }

        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::WHITE);

        let slider_x = WIDTH as f32 - SLIDER_WIDTH_PX - 20.0;
        let slider_y0 = 50.0;

        let mode_button = Rectangle::new(slider_x, slider_y0 - 38.0, SLIDER_WIDTH_PX, 24.0);
        if mouse_pressed && point_in_rect(mouse_pos, mode_button) {
            center_mode = if center_mode == CenterMode::RobotOffset {
                CenterMode::MapCenter
            } else {
                CenterMode::RobotOffset
            };
        }
        if toggle_pressed {
            center_mode = if center_mode == CenterMode::RobotOffset {
                CenterMode::MapCenter
            } else {
                CenterMode::RobotOffset
            };
        }

        update_slider_value(
            Rectangle::new(slider_x, slider_y0, SLIDER_WIDTH_PX, SLIDER_HEIGHT_PX),
            &mut map_scale,
            0.25,
            4.0,
            mouse_pos,
            mouse_down,
        );

        match center_mode {
            CenterMode::RobotOffset => {
                update_slider_value(
                    Rectangle::new(
                        slider_x,
                        slider_y0 + SLIDER_GAP_PX,
                        SLIDER_WIDTH_PX,
                        SLIDER_HEIGHT_PX,
                    ),
                    &mut robot_offset_x,
                    -2.0,
                    2.0,
                    mouse_pos,
                    mouse_down,
                );

                update_slider_value(
                    Rectangle::new(
                        slider_x,
                        slider_y0 + 2.0 * SLIDER_GAP_PX,
                        SLIDER_WIDTH_PX,
                        SLIDER_HEIGHT_PX,
                    ),
                    &mut robot_offset_y,
                    -2.0,
                    2.0,
                    mouse_pos,
                    mouse_down,
                );
            }
            CenterMode::MapCenter => {
                update_slider_value(
                    Rectangle::new(
                        slider_x,
                        slider_y0 + SLIDER_GAP_PX,
                        SLIDER_WIDTH_PX,
                        SLIDER_HEIGHT_PX,
                    ),
                    &mut map_center_x_slider,
                    -2.0,
                    FIELD_WIDTH_X_METERS + 2.0,
                    mouse_pos,
                    mouse_down,
                );

                update_slider_value(
                    Rectangle::new(
                        slider_x,
                        slider_y0 + 2.0 * SLIDER_GAP_PX,
                        SLIDER_WIDTH_PX,
                        SLIDER_HEIGHT_PX,
                    ),
                    &mut map_center_y_slider,
                    -2.0,
                    FIELD_HEIGHT_Y_METERS + 2.0,
                    mouse_pos,
                    mouse_down,
                );
            }
        }

        let telemetry = TELEMETRY.lock().unwrap().clone();
        let map_center_x = match center_mode {
            CenterMode::RobotOffset => {
                value_as_f32(telemetry.values.get(&KEY_POSITION_X))
                    .unwrap_or(FIELD_WIDTH_X_METERS * 0.5)
                    + robot_offset_x
            }
            CenterMode::MapCenter => map_center_x_slider,
        };
        let map_center_y = match center_mode {
            CenterMode::RobotOffset => {
                value_as_f32(telemetry.values.get(&KEY_POSITION_Y))
                    .unwrap_or(FIELD_HEIGHT_Y_METERS * 0.5)
                    + robot_offset_y
            }
            CenterMode::MapCenter => map_center_y_slider,
        };

        let screen_w = WIDTH as f32;
        let screen_h = HEIGHT as f32;

        let usable_w = (screen_w - 2.0 * VIEW_MARGIN_PX).max(1.0);
        let usable_h = (screen_h - 2.0 * VIEW_MARGIN_PX).max(1.0);

        let base_pixels_per_meter =
            (usable_w / FIELD_WIDTH_X_METERS).min(usable_h / FIELD_HEIGHT_Y_METERS);
        let pixels_per_meter = base_pixels_per_meter * map_scale;
        let screen_center_x = screen_w * 0.5;
        let screen_center_y = screen_h * 0.5;

        let world_to_screen = |x_m: f32, y_m: f32| -> Vector2 {
            Vector2::new(
                screen_center_x + (x_m - map_center_x) * pixels_per_meter,
                screen_center_y - (y_m - map_center_y) * pixels_per_meter,
            )
        };

        let screen_to_world = |screen: Vector2| -> Vector2 {
            Vector2::new(
                map_center_x + (screen.x - screen_center_x) / pixels_per_meter,
                map_center_y - (screen.y - screen_center_y) / pixels_per_meter,
            )
        };

        if clear_tracker_pressed {
            tracking_circle = None;
        }

        if right_mouse_pressed {
            let center_world = screen_to_world(mouse_pos);
            tracking_circle = Some(TrackingCircle::new(center_world, 0.05));
            target_cycle_index = 0;
        }

        let bl = world_to_screen(0.0, 0.0);
        let br = world_to_screen(FIELD_WIDTH_X_METERS, 0.0);
        let tr = world_to_screen(FIELD_WIDTH_X_METERS, FIELD_HEIGHT_Y_METERS);
        let tl = world_to_screen(0.0, FIELD_HEIGHT_Y_METERS);

        d.draw_line_ex(bl, br, 2.0, Color::BLACK);
        d.draw_line_ex(br, tr, 2.0, Color::BLACK);
        d.draw_line_ex(tr, tl, 2.0, Color::BLACK);
        d.draw_line_ex(tl, bl, 2.0, Color::BLACK);

        if let Some(grid) = telemetry.occupancy_grid.as_ref() {
            let tile_size_m = grid.tile_size_mm as f32 / 1000.0;

            for y in 0..grid.height as usize {
                for x in 0..grid.width as usize {
                    let index = y * grid.width as usize + x;
                    let score = grid.scores[index];

                    if score == 127 {
                        continue;
                    }

                    let world_x = x as f32 * tile_size_m;
                    let world_y = y as f32 * tile_size_m;

                    let p0 = world_to_screen(world_x, world_y);
                    let p1 = world_to_screen(world_x + tile_size_m, world_y + tile_size_m);

                    let rect = Rectangle::new(
                        p0.x.min(p1.x),
                        p0.y.min(p1.y),
                        (p1.x - p0.x).abs().max(1.0),
                        (p1.y - p0.y).abs().max(1.0),
                    );

                    let color = if score > 127 {
                        let strength = ((score as i32 - 127) as f32 / 128.0).clamp(0.0, 1.0);
                        Color::new(220, 45, 45, (30.0 + 120.0 * strength) as u8)
                    } else {
                        let strength = ((127 - score as i32) as f32 / 127.0).clamp(0.0, 1.0);
                        Color::new(45, 180, 45, (20.0 + 90.0 * strength) as u8)
                    };

                    d.draw_rectangle_rec(rect, color);
                }
            }
        }

        if !telemetry.values.is_empty()
            || !telemetry.lidar_points.is_empty()
            || !telemetry.tracked_weights.is_empty()
        {
            let mut hovered_cluster: Option<HoverCluster> = None;
            let mut hovered_distance_px = f32::INFINITY;
            let mut tuning_candidates: Vec<Cluster> = Vec::new();

            let mut nearest_tracking_cluster: Option<Cluster> = None;
            let mut nearest_tracking_distance_m = f32::INFINITY;

            let heading = value_as_f32(telemetry.values.get(&KEY_HEADING)).unwrap_or(0.0);
            let pitch = value_as_f32(telemetry.values.get(&KEY_PITCH)).unwrap_or(0.0);
            let left_wheel_velocity =
                value_as_f32(telemetry.values.get(&KEY_LEFT_WHEEL_VELOCITY)).unwrap_or(0.0);
            let right_wheel_velocity =
                value_as_f32(telemetry.values.get(&KEY_RIGHT_WHEEL_VELOCITY)).unwrap_or(0.0);
            let position_x = value_as_f32(telemetry.values.get(&KEY_POSITION_X)).unwrap_or(0.0);
            let position_y = value_as_f32(telemetry.values.get(&KEY_POSITION_Y)).unwrap_or(0.0);
            let position_uncertainty =
                value_as_f32(telemetry.values.get(&KEY_POSITION_UNCERTAINTY)).unwrap_or(0.0);

            let drive_error = value_as_f32(telemetry.values.get(&KEY_DRIVE_ERROR)).unwrap_or(0.0);
            let turn_error = value_as_f32(telemetry.values.get(&KEY_TURN_ERROR)).unwrap_or(0.0);

            let left_command = value_as_f32(telemetry.values.get(&KEY_LEFT_COMMAND)).unwrap_or(0.0);
            let right_command =
                value_as_f32(telemetry.values.get(&KEY_RIGHT_COMMAND)).unwrap_or(0.0);

            let nextpoint_x = value_as_f32(telemetry.values.get(&KEY_NEXTPOINT_X)).unwrap_or(0.0);
            let nextpoint_y = value_as_f32(telemetry.values.get(&KEY_NEXTPOINT_Y)).unwrap_or(0.0);

            let lookahead_x = value_as_f32(telemetry.values.get(&KEY_LOOKAHEAD_X)).unwrap_or(0.0);
            let lookahead_y = value_as_f32(telemetry.values.get(&KEY_LOOKAHEAD_Y)).unwrap_or(0.0);

            let weight_target_x = value_as_f32(telemetry.values.get(&KEY_WEIGHT_TARGET_X));
            let weight_target_y = value_as_f32(telemetry.values.get(&KEY_WEIGHT_TARGET_Y));

            let weight_target_confidence =
                value_as_f32(telemetry.values.get(&KEY_WEIGHT_CONFIDENCE));

            let weight_target_spread = value_as_f32(telemetry.values.get(&KEY_WEIGHT_SPREAD));
            let weight_target_extent = value_as_f32(telemetry.values.get(&KEY_WEIGHT_EXTENT));
            let weight_target_diameter = value_as_f32(telemetry.values.get(&KEY_WEIGHT_DIAMETER));
            let weight_target_aspect = value_as_f32(telemetry.values.get(&KEY_WEIGHT_ASPECT));
            let weight_target_range = value_as_f32(telemetry.values.get(&KEY_WEIGHT_RANGE));
            let weight_target_count = value_as_f32(telemetry.values.get(&KEY_WEIGHT_POINT_COUNT));

            if let (
                Some(weight_target_x),
                Some(weight_target_y),
                Some(weight_target_confidence),
                Some(weight_target_spread),
                Some(weight_target_extent),
                Some(weight_target_diameter),
                Some(weight_target_aspect),
                Some(weight_target_range),
                Some(weight_target_count),
            ) = (
                weight_target_x,
                weight_target_y,
                weight_target_confidence,
                weight_target_spread,
                weight_target_extent,
                weight_target_diameter,
                weight_target_aspect,
                weight_target_range,
                weight_target_count,
            ) {
                let weight_target_screen = world_to_screen(weight_target_x, weight_target_y);
                d.draw_circle_v(weight_target_screen, 6.0, Color::PURPLE);
                d.draw_circle_lines_v(weight_target_screen, 12.0, Color::PURPLE);

                d.draw_text(
                    format!("Weight Confidence: {}", weight_target_confidence).as_str(),
                    20,
                    220,
                    20,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("Weight spread: {}", weight_target_spread).as_str(),
                    20,
                    240,
                    20,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("Weight extent: {}", weight_target_extent).as_str(),
                    20,
                    260,
                    20,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("Weight diameter: {}", weight_target_diameter).as_str(),
                    20,
                    280,
                    20,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("Weight aspect: {}", weight_target_aspect).as_str(),
                    20,
                    300,
                    20,
                    Color::BLACK,
                );

                d.draw_text(
                    format!("Weight range: {}", weight_target_range).as_str(),
                    20,
                    320,
                    20,
                    Color::BLACK,
                );

                d.draw_text(
                    format!("Weight point count: {}", weight_target_count).as_str(),
                    20,
                    340,
                    20,
                    Color::BLACK,
                );
            }

            for point in telemetry.lidar_points {
                let lidar_x_world_m = point.x_mm as f32 / 1000.0;
                let lidar_y_world_m = point.y_mm as f32 / 1000.0;

                let screen = world_to_screen(lidar_x_world_m, lidar_y_world_m);

                d.draw_circle_v(
                    screen,
                    2.0,
                    Color::color_from_hsv(360.0 * point.intensity as f32 / 256.0, 1.0, 1.0),
                );
            }

            for line in telemetry.lidar_processing.line_fits {
                let start_screen = world_to_screen(line.x1, line.y1);
                let end_screen = world_to_screen(line.x2, line.y2);

                d.draw_line_ex(start_screen, end_screen, 2.0, Color::GREEN);
            }

            for tracked_weight in telemetry.tracked_weights.iter().copied() {
                let tracked_weight_screen = world_to_screen(tracked_weight.x_m, tracked_weight.y_m);

                d.draw_circle_lines_v(tracked_weight_screen, 10.0, Color::ORANGE);
                d.draw_circle_v(tracked_weight_screen, 3.0, Color::ORANGE);

                d.draw_text(
                    format!("{:.2}", tracked_weight.confidence).as_str(),
                    (tracked_weight_screen.x + 12.0) as i32,
                    (tracked_weight_screen.y - 8.0) as i32,
                    16,
                    Color::ORANGE,
                );
            }

            for cluster in telemetry.lidar_processing.clusters.iter().copied() {
                let c_screen = world_to_screen(cluster.centroid_x, cluster.centroid_y);
                let radius_m = (cluster.diameter_mm.max(0.0) / 1000.0) * 0.5;

                if radius_m >= 0.05 {
                    continue;
                }

                let dx_px = c_screen.x - mouse_pos.x;
                let dy_px = c_screen.y - mouse_pos.y;
                let mouse_distance_px = (dx_px * dx_px + dy_px * dy_px).sqrt();

                let radius_px = (radius_m * pixels_per_meter).max(2.0);

                d.draw_circle_lines_v(c_screen, radius_px, Color::GREEN);
                tuning_candidates.push(cluster);

                if mouse_distance_px <= radius_px.max(6.0)
                    && mouse_distance_px < hovered_distance_px
                {
                    hovered_distance_px = mouse_distance_px;
                    hovered_cluster = Some(HoverCluster { cluster, radius_m });
                }

                if let Some(tracker) = tracking_circle {
                    let dx_m = cluster.centroid_x - tracker.center_world.x;
                    let dy_m = cluster.centroid_y - tracker.center_world.y;
                    let centroid_distance_m = (dx_m * dx_m + dy_m * dy_m).sqrt();

                    if centroid_distance_m <= tracker.radius_m
                        && centroid_distance_m < nearest_tracking_distance_m
                    {
                        nearest_tracking_distance_m = centroid_distance_m;
                        nearest_tracking_cluster = Some(cluster);
                    }
                }
            }

            if tracking_circle.is_some() && !tuning_candidates.is_empty() {
                if target_cycle_index >= tuning_candidates.len() {
                    target_cycle_index = 0;
                }

                if cycle_target_pressed {
                    target_cycle_index = (target_cycle_index + 1) % tuning_candidates.len();

                    if let Some(tracker) = tracking_circle.as_mut() {
                        let selected = tuning_candidates[target_cycle_index];
                        tracker.center_world =
                            Vector2::new(selected.centroid_x, selected.centroid_y);
                        tracker.radius_m =
                            ((selected.diameter_mm.max(0.0) / 1000.0) * 0.5).max(0.05);
                    }
                }
            }

            if let Some(tracker) = tracking_circle.as_mut() {
                if !summing_paused {
                    if let Some(cluster) = nearest_tracking_cluster {
                        tracker.observe_cluster(cluster);
                    } else {
                        tracker.missed_frames += 1;
                    }
                }

                let tracker_screen =
                    world_to_screen(tracker.center_world.x, tracker.center_world.y);
                d.draw_circle_lines_v(
                    tracker_screen,
                    tracker.radius_m * pixels_per_meter,
                    Color::MAGENTA,
                );
                d.draw_circle_v(tracker_screen, 3.0, Color::MAGENTA);
            }

            if let Some(hover) = hovered_cluster {
                d.draw_text("Hover Cluster", 20, 360, 20, Color::DARKGREEN);
                d.draw_text(
                    format!("count: {}", hover.cluster.count).as_str(),
                    20,
                    380,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("range: {:.3}m", hover.cluster.range).as_str(),
                    20,
                    400,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("spread: {:.4}m", hover.cluster.spread).as_str(),
                    20,
                    420,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("extent: {:.4}m", hover.cluster.max_extent).as_str(),
                    20,
                    440,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("diameter: {:.1}mm", hover.cluster.diameter_mm).as_str(),
                    20,
                    460,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("aspect: {:.3}", hover.cluster.aspect_ratio).as_str(),
                    20,
                    480,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!("radius: {:.1}mm", hover.radius_m * 1000.0).as_str(),
                    20,
                    500,
                    18,
                    Color::BLACK,
                );
            }

            if let Some(tracker) = tracking_circle {
                d.draw_text(
                    "Tracker (RMB place, Backspace clear, Tab switch target, Space pause)",
                    20,
                    530,
                    20,
                    Color::MAROON,
                );
                d.draw_text(
                    if summing_paused {
                        "Summing: Paused"
                    } else {
                        "Summing: Live"
                    },
                    20,
                    545,
                    18,
                    if summing_paused {
                        Color::RED
                    } else {
                        Color::DARKGREEN
                    },
                );
                d.draw_text(
                    format!(
                        "samples: {}  missed: {}",
                        tracker.stats.spread.samples, tracker.missed_frames
                    )
                    .as_str(),
                    20,
                    565,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!(
                        "spread m/std: {:.4} / {:.4} m",
                        tracker.stats.spread.mean(),
                        tracker.stats.spread.stddev()
                    )
                    .as_str(),
                    20,
                    585,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!(
                        "extent m/std: {:.4} / {:.4} m",
                        tracker.stats.extent.mean(),
                        tracker.stats.extent.stddev()
                    )
                    .as_str(),
                    20,
                    605,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!(
                        "diameter m/std: {:.2} / {:.2} mm",
                        tracker.stats.diameter_mm.mean(),
                        tracker.stats.diameter_mm.stddev()
                    )
                    .as_str(),
                    20,
                    625,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!(
                        "aspect m/std: {:.3} / {:.3}",
                        tracker.stats.aspect_ratio.mean(),
                        tracker.stats.aspect_ratio.stddev()
                    )
                    .as_str(),
                    20,
                    645,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!(
                        "range m/std: {:.3} / {:.3} m",
                        tracker.stats.range.mean(),
                        tracker.stats.range.stddev()
                    )
                    .as_str(),
                    20,
                    665,
                    18,
                    Color::BLACK,
                );
                d.draw_text(
                    format!(
                        "count m/std: {:.2} / {:.2}",
                        tracker.stats.point_count.mean(),
                        tracker.stats.point_count.stddev()
                    )
                    .as_str(),
                    20,
                    685,
                    18,
                    Color::BLACK,
                );
            }

            let robot_screen = world_to_screen(position_x, position_y);

            let uncertainty_m = position_uncertainty.max(0.01);
            let uncertainty_radius_px = uncertainty_m * pixels_per_meter;
            d.draw_circle_lines_v(robot_screen, uncertainty_radius_px, Color::RED);

            let heading_length_m = 0.22f32;
            let heading_end_x = position_x + heading_length_m * heading.sin();
            let heading_end_y = position_y + heading_length_m * heading.cos();
            let heading_end = world_to_screen(heading_end_x, heading_end_y);
            d.draw_line_ex(robot_screen, heading_end, 3.0, Color::BLUE);

            d.draw_circle_v(world_to_screen(nextpoint_x, nextpoint_y), 3.0, Color::RED);
            d.draw_circle_v(world_to_screen(lookahead_x, lookahead_y), 3.0, Color::BLUE);

            d.draw_text(
                format!(
                    "Heading: {:.2}°, Pitch: {:.2}°",
                    heading as f64 * RAD2DEG,
                    pitch as f64 * RAD2DEG,
                )
                .as_str(),
                20,
                20,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!(
                    "Left: {:.1}rpm, Right: {:.1}rpm",
                    60.0 * left_wheel_velocity / (2.0 * PI),
                    60.0 * right_wheel_velocity / (2.0 * PI)
                )
                .as_str(),
                20,
                40,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("X: {:.3}m, Y: {:.3}m", 1.0 * position_x, 1.0 * position_y).as_str(),
                20,
                80,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("Position Uncertainty: {:.3}m", position_uncertainty).as_str(),
                20,
                100,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!(
                    "Drive Error: {drive_error:.3}m, Turn Error: {:.3}deg",
                    turn_error * 180.0 / PI
                )
                .as_str(),
                20,
                120,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("Left Command: {left_command:.3}, Right Command: {right_command:.3}",)
                    .as_str(),
                20,
                140,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("Lookahead: ({lookahead_x:.3}, {lookahead_y:.3})").as_str(),
                20,
                160,
                20,
                Color::BLACK,
            );

            d.draw_text(
                format!("Nextpoint: ({nextpoint_x:.3}, {nextpoint_y:.3})").as_str(),
                20,
                180,
                20,
                Color::BLACK,
            );
        } else {
            d.draw_text("No Telemetry", 20, 20, 20, Color::BLACK);
        }

        d.draw_rectangle_rec(mode_button, Color::LIGHTGRAY);
        d.draw_rectangle_lines_ex(mode_button, 1.0, Color::GRAY);
        let mode_label = if center_mode == CenterMode::RobotOffset {
            "Mode: Robot Offset (T)"
        } else {
            "Mode: Map Center (T)"
        };
        d.draw_text(
            mode_label,
            (mode_button.x + 8.0) as i32,
            (mode_button.y + 4.0) as i32,
            16,
            Color::BLACK,
        );

        draw_slider(
            &mut d,
            "Map Scale",
            Rectangle::new(slider_x, slider_y0, SLIDER_WIDTH_PX, SLIDER_HEIGHT_PX),
            &mut map_scale,
            0.25,
            4.0,
        );

        match center_mode {
            CenterMode::RobotOffset => {
                draw_slider(
                    &mut d,
                    "Offset X (m)",
                    Rectangle::new(
                        slider_x,
                        slider_y0 + SLIDER_GAP_PX,
                        SLIDER_WIDTH_PX,
                        SLIDER_HEIGHT_PX,
                    ),
                    &mut robot_offset_x,
                    -2.0,
                    2.0,
                );

                draw_slider(
                    &mut d,
                    "Offset Y (m)",
                    Rectangle::new(
                        slider_x,
                        slider_y0 + 2.0 * SLIDER_GAP_PX,
                        SLIDER_WIDTH_PX,
                        SLIDER_HEIGHT_PX,
                    ),
                    &mut robot_offset_y,
                    -2.0,
                    2.0,
                );
            }
            CenterMode::MapCenter => {
                draw_slider(
                    &mut d,
                    "Center X (m)",
                    Rectangle::new(
                        slider_x,
                        slider_y0 + SLIDER_GAP_PX,
                        SLIDER_WIDTH_PX,
                        SLIDER_HEIGHT_PX,
                    ),
                    &mut map_center_x_slider,
                    -2.0,
                    FIELD_WIDTH_X_METERS + 2.0,
                );

                draw_slider(
                    &mut d,
                    "Center Y (m)",
                    Rectangle::new(
                        slider_x,
                        slider_y0 + 2.0 * SLIDER_GAP_PX,
                        SLIDER_WIDTH_PX,
                        SLIDER_HEIGHT_PX,
                    ),
                    &mut map_center_y_slider,
                    -2.0,
                    FIELD_HEIGHT_Y_METERS + 2.0,
                );
            }
        }
    }

    Ok(())
}
