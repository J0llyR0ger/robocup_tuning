use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

pub const TELEMETRY_PREAMBLE: [u8; 4] = [0xAB, 0xCD, 0xEF, 0x42];
pub const TELEMETRY_VERSION: u8 = 1;

pub const FRAME_TYPE_VALUES: u8 = 1;
pub const FRAME_TYPE_LIDAR: u8 = 2;
pub const FRAME_TYPE_LIDAR_PROCESSING: u8 = 3;
pub const FRAME_TYPE_OCCUPANCY_GRID: u8 = 4;
pub const FRAME_TYPE_TRACKED_WEIGHTS: u8 = 5;
pub const FRAME_TYPE_GRID_PATH: u8 = 6;

pub const VALUE_TYPE_FLOAT32: u8 = 1;
pub const VALUE_TYPE_INT32: u8 = 2;
pub const VALUE_TYPE_UINT32: u8 = 3;
pub const VALUE_TYPE_BOOL: u8 = 4;

pub const KEY_HEADING: u16 = 1;
pub const KEY_PITCH: u16 = 2;
pub const KEY_LEFT_WHEEL_VELOCITY: u16 = 3;
pub const KEY_RIGHT_WHEEL_VELOCITY: u16 = 4;
pub const KEY_POSITION_X: u16 = 5;
pub const KEY_POSITION_Y: u16 = 6;
pub const KEY_POSITION_UNCERTAINTY: u16 = 7;
pub const KEY_DRIVE_ERROR: u16 = 8;
pub const KEY_TURN_ERROR: u16 = 9;
pub const KEY_LEFT_COMMAND: u16 = 10;
pub const KEY_RIGHT_COMMAND: u16 = 11;
pub const KEY_LOOKAHEAD_X: u16 = 12;
pub const KEY_LOOKAHEAD_Y: u16 = 13;
pub const KEY_NEXTPOINT_X: u16 = 14;
pub const KEY_NEXTPOINT_Y: u16 = 15;
pub const KEY_WEIGHT_TARGET_X: u16 = 16;
pub const KEY_WEIGHT_TARGET_Y: u16 = 17;
pub const KEY_WEIGHT_CONFIDENCE: u16 = 18;
pub const KEY_WEIGHT_SPREAD: u16 = 19;
pub const KEY_WEIGHT_EXTENT: u16 = 20;
pub const KEY_WEIGHT_DIAMETER: u16 = 21;
pub const KEY_WEIGHT_ASPECT: u16 = 22;
pub const KEY_WEIGHT_RANGE: u16 = 23;
pub const KEY_WEIGHT_POINT_COUNT: u16 = 24;

pub const KEY_WEIGHT_TUNING_SAMPLE_COUNT: u16 = 100;
pub const KEY_WEIGHT_TUNING_RANGE_BASE: u16 = 128;
pub const KEY_WEIGHT_TUNING_SPREAD_DESIRED_BASE: u16 = 160;
pub const KEY_WEIGHT_TUNING_SPREAD_DEVIATION_BASE: u16 = 192;
pub const KEY_WEIGHT_TUNING_SPREAD_WEIGHT_BASE: u16 = 224;
pub const KEY_WEIGHT_TUNING_EXTENT_DESIRED_BASE: u16 = 256;
pub const KEY_WEIGHT_TUNING_EXTENT_DEVIATION_BASE: u16 = 288;
pub const KEY_WEIGHT_TUNING_EXTENT_WEIGHT_BASE: u16 = 320;
pub const KEY_WEIGHT_TUNING_DIAMETER_DESIRED_BASE: u16 = 352;
pub const KEY_WEIGHT_TUNING_DIAMETER_DEVIATION_BASE: u16 = 384;
pub const KEY_WEIGHT_TUNING_DIAMETER_WEIGHT_BASE: u16 = 416;
pub const KEY_WEIGHT_TUNING_ASPECT_DESIRED_BASE: u16 = 448;
pub const KEY_WEIGHT_TUNING_ASPECT_DEVIATION_BASE: u16 = 480;
pub const KEY_WEIGHT_TUNING_ASPECT_WEIGHT_BASE: u16 = 512;

pub const TELEMETRY_FRAME_HEADER_LEN: usize = 8;

#[derive(Copy, Clone, Debug)]
pub struct ValueEntry {
    pub key: u16,
    pub value_type: u8,
    pub payload: u32,
}

#[derive(Copy, Clone, Debug)]
pub struct LidarPoint {
    pub x_mm: i16,
    pub y_mm: i16,
    pub intensity: u8,
    pub flags: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, FromBytes, KnownLayout, Immutable)]
pub struct LineFit {
    pub x1: f32,
    pub x2: f32,
    pub y1: f32,
    pub y2: f32,

    pub slope: f32,
    pub intercept: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, FromBytes, KnownLayout, Immutable)]
pub struct Cluster {
    pub start: u16,
    pub count: u16,
    pub centroid_x: f32,
    pub centroid_y: f32,
    pub range: f32,
    pub spread: f32,
    pub max_extent: f32,
    pub diameter_mm: f32,
    pub aspect_ratio: f32,
}

#[derive(Clone, Debug)]
pub struct LidarProcessing {
    pub line_fits: Vec<LineFit>,
    pub clusters: Vec<Cluster>,
}

#[derive(Clone, Debug)]
pub struct OccupancyGrid {
    pub width: u16,
    pub height: u16,
    pub tile_size_mm: u16,
    pub scores: Vec<u8>,
}

#[derive(Copy, Clone, Debug)]
pub struct TrackedWeight {
    pub x_m: f32,
    pub y_m: f32,
    pub confidence: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct GridPathPoint {
    pub x_m: f32,
    pub y_m: f32,
}

#[derive(Clone, Debug)]
pub enum TelemetryFrame {
    Values(Vec<ValueEntry>),
    Lidar(Vec<LidarPoint>),
    LidarProcessing(LidarProcessing),
    OccupancyGrid(OccupancyGrid),
    TrackedWeights(Vec<TrackedWeight>),
    GridPath(Vec<GridPathPoint>),
}

pub fn parse_frame(frame_type: u8, payload: &[u8]) -> Option<TelemetryFrame> {
    if payload.len() < 2 {
        return None;
    }

    match frame_type {
        FRAME_TYPE_VALUES => {
            let count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let body = &payload[2..];

            let entry_size = 8usize;
            if body.len() < count * entry_size {
                return None;
            }

            let mut entries = Vec::with_capacity(count);
            for i in 0..count {
                let base = i * entry_size;
                let key = u16::from_le_bytes([body[base], body[base + 1]]);
                let value_type = body[base + 2];
                let payload = u32::from_le_bytes([
                    body[base + 4],
                    body[base + 5],
                    body[base + 6],
                    body[base + 7],
                ]);

                entries.push(ValueEntry {
                    key,
                    value_type,
                    payload,
                });
            }

            Some(TelemetryFrame::Values(entries))
        }
        FRAME_TYPE_LIDAR => {
            let count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let body = &payload[2..];

            let point_size = 6usize;
            if body.len() < count * point_size {
                return None;
            }

            let mut points = Vec::with_capacity(count);
            for i in 0..count {
                let base = i * point_size;
                let x_mm = i16::from_le_bytes([body[base], body[base + 1]]);
                let y_mm = i16::from_le_bytes([body[base + 2], body[base + 3]]);
                let intensity = body[base + 4];
                let flags = body[base + 5];

                points.push(LidarPoint {
                    x_mm,
                    y_mm,
                    intensity,
                    flags,
                });
            }

            Some(TelemetryFrame::Lidar(points))
        }
        FRAME_TYPE_LIDAR_PROCESSING => {
            const LINE_SIZE: usize = size_of::<LineFit>();
            const EXPECTED_LINE_SIZE: usize = 24;
            if LINE_SIZE != EXPECTED_LINE_SIZE {
                return None;
            }

            if payload.len() < 2 {
                return None;
            }

            let lines_count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let lines_body_len = lines_count * LINE_SIZE;

            if payload.len() < 2 + lines_body_len + 2 {
                return None;
            }

            let lines_body = &payload[2..2 + lines_body_len];

            let mut lines = Vec::with_capacity(lines_count);

            for i in 0..lines_count {
                let data =
                    LineFit::read_from_bytes(&lines_body[i * LINE_SIZE..i * LINE_SIZE + LINE_SIZE])
                        .ok()?;

                lines.push(data);
            }

            const CLUSTER_SIZE: usize = size_of::<Cluster>();
            const EXPECTED_CLUSTER_SIZE: usize = 32;
            if CLUSTER_SIZE != EXPECTED_CLUSTER_SIZE {
                return None;
            }

            let clusters_count =
                u16::from_le_bytes([payload[2 + lines_body_len], payload[3 + lines_body_len]])
                    as usize;
            let clusters_body = &payload[4 + lines_body_len..];

            if clusters_body.len() < clusters_count * CLUSTER_SIZE {
                return None;
            }

            let mut clusters = Vec::with_capacity(clusters_count);

            for i in 0..clusters_count {
                let data = Cluster::read_from_bytes(
                    &clusters_body[i * CLUSTER_SIZE..(i + 1) * CLUSTER_SIZE],
                )
                .ok()?;

                clusters.push(data);
            }

            Some(TelemetryFrame::LidarProcessing(LidarProcessing {
                line_fits: lines,
                clusters,
            }))
        }
        FRAME_TYPE_OCCUPANCY_GRID => {
            if payload.len() < 6 {
                return None;
            }

            let width = u16::from_le_bytes([payload[0], payload[1]]);
            let height = u16::from_le_bytes([payload[2], payload[3]]);
            let tile_size_mm = u16::from_le_bytes([payload[4], payload[5]]);

            let cell_count = width as usize * height as usize;
            if payload.len() < 6 + cell_count {
                return None;
            }

            let scores = payload[6..6 + cell_count].to_vec();

            Some(TelemetryFrame::OccupancyGrid(OccupancyGrid {
                width,
                height,
                tile_size_mm,
                scores,
            }))
        }
        FRAME_TYPE_TRACKED_WEIGHTS => {
            let count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let body = &payload[2..];

            let entry_size = 12usize;
            if body.len() < count * entry_size {
                return None;
            }

            let mut tracked_weights = Vec::with_capacity(count);
            for i in 0..count {
                let base = i * entry_size;

                let x_m = f32::from_le_bytes([
                    body[base],
                    body[base + 1],
                    body[base + 2],
                    body[base + 3],
                ]);
                let y_m = f32::from_le_bytes([
                    body[base + 4],
                    body[base + 5],
                    body[base + 6],
                    body[base + 7],
                ]);
                let confidence = f32::from_le_bytes([
                    body[base + 8],
                    body[base + 9],
                    body[base + 10],
                    body[base + 11],
                ]);

                tracked_weights.push(TrackedWeight {
                    x_m,
                    y_m,
                    confidence,
                });
            }

            Some(TelemetryFrame::TrackedWeights(tracked_weights))
        }
        FRAME_TYPE_GRID_PATH => {
            let count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let body = &payload[2..];

            let entry_size = 8usize;
            if body.len() < count * entry_size {
                return None;
            }

            let mut path_points = Vec::with_capacity(count);
            for i in 0..count {
                let base = i * entry_size;

                let x_m = f32::from_le_bytes([
                    body[base],
                    body[base + 1],
                    body[base + 2],
                    body[base + 3],
                ]);
                let y_m = f32::from_le_bytes([
                    body[base + 4],
                    body[base + 5],
                    body[base + 6],
                    body[base + 7],
                ]);

                path_points.push(GridPathPoint { x_m, y_m });
            }

            Some(TelemetryFrame::GridPath(path_points))
        }
        _ => None,
    }
}

#[repr(C, packed)]
#[derive(Copy, Clone, FromBytes, IntoBytes, Debug, KnownLayout, Immutable)]
pub struct CommandPacket {
    pub left_command: i8,
    pub right_command: i8,
}

pub const COMMAND_HEADER: [u8; 10] = [0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00];
