use std::f32::consts::PI;

use raylib::prelude::*;
use zerocopy::IntoBytes;

use crate::{
    data_source::CommandSink,
    protocol::{
        COMMAND_HEADER, CommandPacket, KEY_DRIVE_ERROR, KEY_HEADING, KEY_LEFT_COMMAND,
        KEY_LEFT_WHEEL_VELOCITY, KEY_LOOKAHEAD_X, KEY_LOOKAHEAD_Y, KEY_NEXTPOINT_X,
        KEY_NEXTPOINT_Y, KEY_PITCH, KEY_POSITION_UNCERTAINTY, KEY_POSITION_X, KEY_POSITION_Y,
        KEY_RIGHT_COMMAND, KEY_RIGHT_WHEEL_VELOCITY, KEY_TURN_ERROR, KEY_WEIGHT_ASPECT,
        KEY_WEIGHT_CONFIDENCE, KEY_WEIGHT_DIAMETER, KEY_WEIGHT_EXTENT, KEY_WEIGHT_RANGE,
        KEY_WEIGHT_SPREAD, KEY_WEIGHT_TARGET_X, KEY_WEIGHT_TARGET_Y, VALUE_TYPE_FLOAT32,
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
        let toggle_pressed = rl.is_key_pressed(KeyboardKey::KEY_T);
        prev_mouse_down = mouse_down;

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

        if !telemetry.values.is_empty() || !telemetry.lidar_points.is_empty() {
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

            if let (
                Some(weight_target_x),
                Some(weight_target_y),
                Some(weight_target_confidence),
                Some(weight_target_spread),
                Some(weight_target_extent),
                Some(weight_target_diameter),
                Some(weight_target_aspect),
                Some(weight_target_range),
            ) = (
                weight_target_x,
                weight_target_y,
                weight_target_confidence,
                weight_target_spread,
                weight_target_extent,
                weight_target_diameter,
                weight_target_aspect,
                weight_target_range,
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

            for circle in telemetry.lidar_processing.circle_fits {
                let c_screen = world_to_screen(circle.cx, circle.cy);

                d.draw_circle_v(c_screen, circle.r * pixels_per_meter, Color::GREEN);
                d.draw_circle_v(c_screen, circle.r * pixels_per_meter - 5.0, Color::WHITE);

                d.draw_text(
                    format!("Radius: {}", circle.r).as_str(),
                    c_screen.x as i32,
                    c_screen.y as i32,
                    20,
                    Color::BLACK,
                )
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
