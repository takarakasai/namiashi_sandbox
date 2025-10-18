use nalgebra::Vector3;
use serialport::{DataBits, FlowControl, Parity, StopBits};
use std::time::Duration;
use std::io::{self, Read};
use chrono::Local;

#[derive(Debug)]
struct SBus2 {
  ch: [f32; 16],
  failsafe: bool,
  framelost: bool,
  ch18: bool,
  flag: bool,
}

#[derive(Debug)]
struct Twist {
  linear: Vector3<f32>,
  angular: Vector3<f32>
}

#[derive(Debug)]
enum ParseError {
  InvalidLength,
  ChecksumMismatch,
  IllegalFrame,
}

fn main() -> io::Result<()> {
  // シリアルポートの設定
  // let port_name = "/dev/ttyUSB0"; // 使用するポートを指定してください (例: COM3, /dev/ttyUSB0)
  // let port_name = "/dev/ttyS0"; // 使用するポートを指定してください (例: COM3, /dev/ttyUSB0)
  let port_name = "/dev/ttyAMA4"; // 使用するポートを指定してください (例: COM3, /dev/ttyUSB0)
  let baud_rate = 115200;

  let mut port = serialport::new(port_name, baud_rate)
      .data_bits(DataBits::Eight)
      .parity(Parity::None)
      .stop_bits(StopBits::One)
      .flow_control(FlowControl::None)
      .timeout(Duration::from_millis(1)) // タイムアウトを1秒に設定
      .open()
      .expect("ポートを開けませんでした");

  println!("シリアルポート {} に接続しました", port_name);

  let mut buffer: Vec<u8> = vec![0; 1024]; // 読み取り用バッファ
  let mut frame_buffer: Vec<u8> = Vec::new(); // フレームを蓄積するバッファ
  let mut count : i64 = 0;

  loop {
    match port.read(&mut buffer) {
      Ok(bytes_read) => {
        if bytes_read > 0 {
          let now = Local::now();
          // println!("{} : {} ({})", bytes_read, { let tmp = count; count += 1; tmp }, now.format("%S%.3f"));
          for &byte in &buffer[..bytes_read] {
            if byte == 0x0F {
              if !frame_buffer.is_empty() {
                let hex_data = frame_buffer
                  .iter()
                  .map(|b| format!("{:02X}", b))
                  .collect::<Vec<String>>();

                let sbus2 = parse_frame(&frame_buffer);
                match sbus2 {
                  Ok(result) => dump_frame(&result),
                  Err(_)     => (),
                }

                frame_buffer.clear();
              }
            }
            frame_buffer.push(byte);
          }
        }
      }
      Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
        // タイムアウト時は何もしない
      }
      Err(e) => {
        eprintln!("エラー: {:?}", e);
        break;
      }
    }
  }

  Ok(())
}

fn parse_frame(frame: &[u8]) -> Result<SBus2, ParseError> {
  let mut chs = [0f32; 16];

  if frame.len() < 35 {
    println!("frame is too short len:{}", frame.len());
    return Err(ParseError::InvalidLength);
  }

  let sum = frame[34];
  let calculated_checksum = calculate_checksum(&frame[1..34]);
  if sum != calculated_checksum {
    println!("checksum mismatch: frame:{:02x} vs calc:{:02x}", sum, calculated_checksum);
    return Err(ParseError::ChecksumMismatch);
  }

  // 2バイトごとに区切って符号なし整数に変換
  for (idx, chunk) in frame[1..33].chunks(2).enumerate() {
    if chunk.len() == 2 {
      let value = u16::from_be_bytes([chunk[0], chunk[1]]); // ビッグエンディアンで変換
      let mut dvalue = (value as i32 - 1024) as f32 / 850f32;
      if dvalue > 0.99 {
        dvalue =  1.0;
      } else if dvalue < -0.99 { 
        dvalue = -1.0;
      } else {
      }
      chs[idx] = dvalue;
    } else {
      println!("illegal frame: {:?}", chunk); // データが2バイト未満の場合
      return Err(ParseError::IllegalFrame);
    }
  }

  let b23 = frame[33];
  let failsafe  = (b23 & 0b0000_1000) != 0; 
  let framelost = (b23 & 0b0000_0100) != 0; 
  let ch18      = (b23 & 0b0000_0010) != 0; 
  let flag      = (b23 & 0b0000_0001) != 0; 

  Ok(SBus2 { ch: chs, failsafe: failsafe, framelost: framelost, ch18: ch18, flag: flag })
}

fn dump_frame(frame: &SBus2) {
  frame.ch.iter().enumerate().for_each(|(idx, &ch)| println!("ch{:02}: {:+6.4}", idx, ch));
  println!("FS:{} FL:{} CH18:{} FLG:{}",
    frame.failsafe, frame.framelost, frame.ch18, frame.flag);
}

fn calculate_checksum(data: &[u8]) -> u8 {
  data.iter().fold(0u8, |acc, &byte| acc ^ byte)
}
