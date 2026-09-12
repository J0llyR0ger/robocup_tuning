use std::{thread, time::Duration};

use crate::{
    BAUD_RATE,
    protocol::{
        TELEMETRY_FRAME_HEADER_LEN, TELEMETRY_PREAMBLE, TELEMETRY_VERSION, TelemetryFrame,
        parse_frame,
    },
    telemetry_state::{TELEMETRY, TypedValue},
};
use futures_util::{SinkExt, StreamExt};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc;
use tokio_serial::SerialPortBuilderExt;
use tokio_tungstenite::{connect_async, tungstenite::Message};

const MAX_INCOMING_STREAM_BYTES: usize = 256 * 1024;

pub enum DataSource {
    Serial(String),
    WebSocket(String),
}

pub enum CommandSink {
    Serial(mpsc::UnboundedSender<Vec<u8>>),
    WebSocket(mpsc::UnboundedSender<Vec<u8>>),
    None,
}

/// Starts communication through either direct serial, or via a websocket
///
/// Returns a CommandSink for sending data to the Teensy
pub fn initialize_data_source(
    data_source: DataSource,
) -> Result<CommandSink, Box<dyn std::error::Error>> {
    match data_source {
        DataSource::Serial(serial_port) => start_serial_mode(&serial_port),
        DataSource::WebSocket(url) => start_websocket_mode(&url),
    }
}

/// Starts direct serial communication with the Teensy
///
/// Returns a CommandSink for sending data do the Teensy
fn start_serial_mode(serial_port: &str) -> Result<CommandSink, Box<dyn std::error::Error>> {
    let serial_port = serial_port.to_string();
    let (command_tx, command_rx) = mpsc::unbounded_channel::<Vec<u8>>();

    thread::spawn(move || {
        let runtime = match tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
        {
            Ok(runtime) => runtime,
            Err(error) => {
                eprintln!("Failed to create Tokio runtime: {error}");
                return;
            }
        };

        runtime.block_on(run_serial_client(serial_port, command_rx));
    });

    Ok(CommandSink::Serial(command_tx))
}

async fn run_serial_client(serial_port: String, mut command_rx: mpsc::UnboundedReceiver<Vec<u8>>) {
    let mut incoming_stream = Vec::<u8>::with_capacity(32 * 1024);

    loop {
        match tokio_serial::new(serial_port.as_str(), BAUD_RATE).open_native_async() {
            Ok(serial) => {
                let (mut serial_reader, mut serial_writer) = tokio::io::split(serial);

                let mut read_buf = [0u8; 4096];
                let mut should_reconnect = false;

                while !should_reconnect {
                    tokio::select! {
                        command = command_rx.recv() => {
                            match command {
                                Some(payload) => {
                                    if let Err(error) = serial_writer.write_all(payload.as_slice()).await {
                                        eprintln!("Serial write error: {error}");
                                        should_reconnect = true;
                                    } else if let Err(error) = serial_writer.flush().await {
                                        eprintln!("Serial flush error: {error}");
                                        should_reconnect = true;
                                    }
                                }
                                None => return,
                            }
                        }
                        read_result = serial_reader.read(&mut read_buf) => {
                            match read_result {
                                Ok(0) => {}
                                Ok(read_count) => {
                                    incoming_stream.extend_from_slice(&read_buf[..read_count]);

                                    consume_stream_frames(&mut incoming_stream);
                                }
                                Err(error) => {
                                    eprintln!("Serial read error: {error}");
                                    should_reconnect = true;
                                }
                            }
                        }
                    }
                }
            }
            Err(error) => {
                eprintln!("Serial open error: {error}");
            }
        }

        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}

fn start_websocket_mode(url: &str) -> Result<CommandSink, Box<dyn std::error::Error>> {
    let url = url.to_string();
    let (command_tx, command_rx) = mpsc::unbounded_channel::<Vec<u8>>();

    thread::spawn(move || {
        let runtime = match tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
        {
            Ok(runtime) => runtime,
            Err(error) => {
                eprintln!("Failed to create Tokio runtime: {error}");
                return;
            }
        };

        runtime.block_on(run_websocket_client(url, command_rx));
    });

    Ok(CommandSink::WebSocket(command_tx))
}

async fn run_websocket_client(url: String, mut command_rx: mpsc::UnboundedReceiver<Vec<u8>>) {
    let mut incoming_serial_stream = Vec::<u8>::with_capacity(16 * 1024);

    loop {
        match connect_async(url.as_str()).await {
            Ok((websocket, _)) => {
                println!("Connected to telemetry bridge at {url}");
                let (mut websocket_writer, mut websocket_reader) = websocket.split();

                let mut should_reconnect = false;

                while !should_reconnect {
                    tokio::select! {
                        command = command_rx.recv() => {
                            match command {
                                Some(payload) => {
                                    if let Err(error) = websocket_writer.send(Message::Binary(payload.into())).await {
                                        eprintln!("Websocket send error: {error}");
                                        should_reconnect = true;
                                    }
                                }
                                None => return,
                            }
                        }
                        incoming = websocket_reader.next() => {
                            match incoming {
                                Some(Ok(Message::Binary(payload))) => {
                                    incoming_serial_stream.extend_from_slice(payload.as_ref());

                                    if incoming_serial_stream.len() > MAX_INCOMING_STREAM_BYTES {
                                        let keep = TELEMETRY_FRAME_HEADER_LEN + TELEMETRY_PREAMBLE.len();
                                        let drop_len = incoming_serial_stream.len().saturating_sub(keep);
                                        incoming_serial_stream.drain(..drop_len);
                                    }

                                    consume_stream_frames(&mut incoming_serial_stream);
                                }
                                Some(Ok(Message::Close(_))) => {
                                    eprintln!("Websocket closed by peer");
                                    should_reconnect = true;
                                }
                                Some(Ok(_)) => {}
                                Some(Err(error)) => {
                                    eprintln!("Websocket receive error: {error}");
                                    should_reconnect = true;
                                }
                                None => {
                                    should_reconnect = true;
                                }
                            }
                        }
                    }
                }
            }
            Err(error) => {
                eprintln!("Websocket connect error: {error}");
            }
        }

        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}

fn consume_stream_frames(stream: &mut Vec<u8>) {
    while let Some(frame) = extract_telemetry_frame(stream) {
        apply_frame(frame);
    }
}

fn extract_telemetry_frame(stream: &mut Vec<u8>) -> Option<TelemetryFrame> {
    let preamble_len = TELEMETRY_PREAMBLE.len();

    let preamble_pos = stream
        .windows(preamble_len)
        .position(|window| window == TELEMETRY_PREAMBLE.as_slice());

    let preamble_pos = match preamble_pos {
        Some(position) => position,
        None => {
            let keep_len = preamble_len.saturating_sub(1);
            if stream.len() > keep_len {
                let start = stream.len() - keep_len;
                stream.drain(..start);
            }
            return None;
        }
    };

    if preamble_pos > 0 {
        stream.drain(..preamble_pos);
    }

    if stream.len() < TELEMETRY_FRAME_HEADER_LEN {
        return None;
    }

    let frame_type = stream[4];
    let version = stream[5];

    if version != TELEMETRY_VERSION {
        stream.drain(..1);
        return None;
    }

    let payload_len = u16::from_le_bytes([stream[6], stream[7]]) as usize;
    let frame_len = TELEMETRY_FRAME_HEADER_LEN + payload_len;

    if stream.len() < frame_len {
        return None;
    }

    let frame = parse_frame(frame_type, &stream[TELEMETRY_FRAME_HEADER_LEN..frame_len]);
    stream.drain(..frame_len);
    frame
}

fn apply_frame(frame: TelemetryFrame) {
    let mut telemetry = TELEMETRY.lock().unwrap();

    match frame {
        TelemetryFrame::Values(entries) => {
            for entry in entries {
                telemetry.values.insert(
                    entry.key,
                    TypedValue {
                        value_type: entry.value_type,
                        payload: entry.payload,
                    },
                );
            }
        }
        TelemetryFrame::Lidar(points) => {
            telemetry.lidar_points = points;
        }
        TelemetryFrame::LidarProcessing(processing) => {
            telemetry.lidar_processing = processing;
        }
        TelemetryFrame::OccupancyGrid(grid) => {
            telemetry.occupancy_grid = Some(grid);
        }
        TelemetryFrame::TrackedWeights(tracked_weights) => {
            telemetry.tracked_weights = tracked_weights;
        }
    }
}
