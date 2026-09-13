use std::{
    collections::HashMap,
    sync::{LazyLock, Mutex},
};

use crate::protocol::{LidarPoint, LidarProcessing, OccupancyGrid, TrackedWeight};

#[derive(Copy, Clone, Debug)]
pub struct TypedValue {
    pub value_type: u8,
    pub payload: u32,
}

#[derive(Clone, Debug)]
pub struct TelemetrySnapshot {
    pub values: HashMap<u16, TypedValue>,
    pub lidar_points: Vec<LidarPoint>,
    pub lidar_processing: LidarProcessing,
    pub occupancy_grid: Option<OccupancyGrid>,
    pub tracked_weights: Vec<TrackedWeight>,
    pub grid_path: Vec<u16>,
}

impl TelemetrySnapshot {
    pub fn new() -> Self {
        Self {
            values: HashMap::new(),
            lidar_points: Vec::new(),
            lidar_processing: LidarProcessing {
                line_fits: vec![],
                clusters: vec![],
            },
            occupancy_grid: None,
            tracked_weights: Vec::new(),
            grid_path: Vec::new(),
        }
    }
}

pub static TELEMETRY: LazyLock<Mutex<TelemetrySnapshot>> =
    LazyLock::new(|| Mutex::new(TelemetrySnapshot::new()));
