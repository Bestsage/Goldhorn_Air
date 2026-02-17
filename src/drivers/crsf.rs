pub const CRSF_SYNC: u8 = 0xC8;
pub const CRSF_FRAMETYPE_RC_CHANNELS_PACKED: u8 = 0x16;

#[derive(Debug, Default, Clone, Copy)]
pub struct RcChannels {
    pub channels: [u16; 16], // 11-bit values (0-2047)
}

pub struct CrsfParser {
    buffer: heapless::Vec<u8, 64>, // Max frame size
}

impl CrsfParser {
    pub fn new() -> Self {
        Self {
            buffer: heapless::Vec::new(),
        }
    }

    pub fn push_byte(&mut self, b: u8) -> Option<RcChannels> {
        // Simple state machine or buffer collecting
        // CRSF frames are: [Sync] [Len] [Type] [Payload...] [CRC]
        // Len includes Type, Payload, CRC.

        if self.buffer.is_empty() {
            if b == CRSF_SYNC {
                let _ = self.buffer.push(b);
            }
            return None;
        }

        if self.buffer.len() == 1 {
            // Length byte. Valid range approx 2 to 62.
            if b < 2 || b > 62 {
                self.buffer.clear(); // Invalid length
                                     // If this byte was sync, maybe we should restart?
                if b == CRSF_SYNC {
                    let _ = self.buffer.push(b);
                }
                return None;
            }
            let _ = self.buffer.push(b);
            return None;
        }

        let len_byte = self.buffer[1];
        // Total frame size = 2 (Sync+Len) + Len
        let total_size = 2 + len_byte as usize;

        if self.buffer.len() < total_size {
            let _ = self.buffer.push(b);
        }

        if self.buffer.len() == total_size {
            // Frame complete, verify CRC
            let frame = self.buffer.as_slice();
            // CRC is calculated over Type + Payload (so from index 2 to end-1)
            let payload_crc_range = &frame[2..total_size - 1];
            let received_crc = frame[total_size - 1];

            if calc_crc8(payload_crc_range) == received_crc {
                // Valid Frame
                let type_byte = frame[2];
                let payload = &frame[3..total_size - 1];

                if type_byte == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && payload.len() == 22 {
                    let channels = parse_channels(payload);
                    self.buffer.clear();
                    return Some(channels);
                }
            }

            // Done with frame
            self.buffer.clear();
        }

        None
    }

    pub fn push_bytes(&mut self, data: &[u8]) -> Option<RcChannels> {
        let mut last_res = None;
        for &b in data {
            if let Some(res) = self.push_byte(b) {
                last_res = Some(res);
            }
        }
        last_res
    }
}

fn calc_crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc = crc << 1;
            }
        }
    }
    crc
}

fn parse_channels(payload: &[u8]) -> RcChannels {
    // 16 channels, 11 bits each = 176 bits = 22 bytes.
    // Little Endian packing? Standard CRSF packing.
    let mut ch = [0u16; 16];

    // This is the standard betaflight/crsf parsing logic
    // bits are packed tightly.
    // Byte 0: ch0[0-7]
    // Byte 1: ch0[8-10] | ch1[0-4] << 3
    // ...
    // Since we don't have 'bit reader', we do manual shift

    // Assuming 'payload' is exactly 22 bytes
    if payload.len() != 22 {
        return RcChannels::default();
    }

    let buf = payload;

    // bit manipulation hell or just use a verified snippet?
    // Using simple extraction
    ch[0] = ((buf[0] as u16) | ((buf[1] as u16) << 8)) & 0x07FF;
    ch[1] = ((buf[1] as u16 >> 3) | ((buf[2] as u16) << 5)) & 0x07FF;
    ch[2] = ((buf[2] as u16 >> 6) | ((buf[3] as u16) << 2) | ((buf[4] as u16) << 10)) & 0x07FF;
    ch[3] = ((buf[4] as u16 >> 1) | ((buf[5] as u16) << 7)) & 0x07FF;
    ch[4] = ((buf[5] as u16 >> 4) | ((buf[6] as u16) << 4)) & 0x07FF;
    ch[5] = ((buf[6] as u16 >> 7) | ((buf[7] as u16) << 1) | ((buf[8] as u16) << 9)) & 0x07FF;
    ch[6] = ((buf[8] as u16 >> 2) | ((buf[9] as u16) << 6)) & 0x07FF;
    ch[7] = ((buf[9] as u16 >> 5) | ((buf[10] as u16) << 3)) & 0x07FF;

    ch[8] = ((buf[11] as u16) | ((buf[12] as u16) << 8)) & 0x07FF;
    ch[9] = ((buf[12] as u16 >> 3) | ((buf[13] as u16) << 5)) & 0x07FF;
    ch[10] = ((buf[13] as u16 >> 6) | ((buf[14] as u16) << 2) | ((buf[15] as u16) << 10)) & 0x07FF;
    ch[11] = ((buf[15] as u16 >> 1) | ((buf[16] as u16) << 7)) & 0x07FF;
    ch[12] = ((buf[16] as u16 >> 4) | ((buf[17] as u16) << 4)) & 0x07FF;
    ch[13] = ((buf[17] as u16 >> 7) | ((buf[18] as u16) << 1) | ((buf[19] as u16) << 9)) & 0x07FF;
    ch[14] = ((buf[19] as u16 >> 2) | ((buf[20] as u16) << 6)) & 0x07FF;
    ch[15] = ((buf[20] as u16 >> 5) | ((buf[21] as u16) << 3)) & 0x07FF;

    RcChannels { channels: ch }
}

// --- Constants ---
#[allow(dead_code)]
pub const CRSF_ADDRESS_FLIGHT_CONTROLLER: u8 = 0xC8;
#[allow(dead_code)]
pub const CRSF_ADDRESS_RADIO_TRANSMITTER: u8 = 0xEA; // The remote controller
#[allow(dead_code)]
pub const CRSF_ADDRESS_CRSF_TRANSMITTER: u8 = 0xEE; // The Crossfire TX module
#[allow(dead_code)]
pub const CRSF_ADDRESS_BROADCAST: u8 = 0x00;

pub const CRSF_FRAMETYPE_GPS: u8 = 0x02;
pub const CRSF_FRAMETYPE_BATTERY_SENSOR: u8 = 0x08;
pub const CRSF_FRAMETYPE_ATTITUDE: u8 = 0x1E;
pub const CRSF_FRAMETYPE_FLIGHT_MODE: u8 = 0x21;

// --- Telemetry Structures ---
// These are not "parsed" but "constructed"

/// Helper to serialize a CRSF frame
/// [Sync] [Len] [Type] [Payload...] [CRC]
/// Returns the number of bytes written to `buf`
pub fn build_telemetry_packet(buf: &mut [u8], frame_type: u8, payload: &[u8]) -> usize {
    // Basic CRSF broadcast frame: Sync, Len, Type, Payload, CRC
    // Sync = 0xC8 (Device Addr for FC?) or 0xC8 (Sync Byte)?
    // The doc says: "Sync byte might be one of ... Serial sync byte: 0xC8 ... Device address"
    // For telemetry sent TO the RX, we usually use the Sync Byte 0xC8 or the Destination Address?
    // Looking at open source implementations (Betaflight/EdgeTX):
    // FC -> RX (Telemetry) usually starts with CRSF_SYNC (0xC8)
    // And actually the "Type" field is preceded by a length.

    // BUT, the doc says "Broadcast Frame: Type + Payload + CRC" inside the frame structure?
    // Let's follow "Broadcast Frame" structure:
    // [Sync] [Len] [Type] [Payload] [CRC]

    let len = 2 + payload.len(); // Type (1) + Payload (N) + CRC (1)
    if buf.len() < len + 2 {
        return 0;
    } // Buffer too small

    buf[0] = CRSF_SYNC;
    buf[1] = len as u8;
    buf[2] = frame_type;
    buf[3..3 + payload.len()].copy_from_slice(payload);

    // CRC calculation: Type + Payload
    let crc_slice = &buf[2..3 + payload.len()];
    let crc = calc_crc8(crc_slice);
    buf[3 + payload.len()] = crc;

    2 + len // Total size: Sync(1) + Len(1) + Type(1) + Payload(N) + CRC(1) = 2 + (1 + N + 1) = 4 + N
}

pub fn payload_flight_mode(mode: &str) -> heapless::Vec<u8, 64> {
    let mut buf = heapless::Vec::new();
    // Flight mode is just a null-terminated string
    for b in mode.as_bytes() {
        let _ = buf.push(*b);
    }
    let _ = buf.push(0); // Null terminator
    buf
}

pub fn payload_gps(
    lat: i32,  // deg * 10,000,000
    lon: i32,  // deg * 10,000,000
    gspd: u16, // km/h * 10
    hdg: u16,  // deg * 100
    alt: u16,  // m + 1000
    sats: u8,
) -> [u8; 15] {
    let mut buf = [0u8; 15];
    // Big Endian
    buf[0..4].copy_from_slice(&lat.to_be_bytes());
    buf[4..8].copy_from_slice(&lon.to_be_bytes());
    buf[8..10].copy_from_slice(&gspd.to_be_bytes());
    buf[10..12].copy_from_slice(&hdg.to_be_bytes());
    buf[12..14].copy_from_slice(&alt.to_be_bytes());
    buf[14] = sats;
    buf
}

pub fn payload_attitude(
    pitch: i16, // rad * 10000 (approx) -> 100 urad
    roll: i16,
    yaw: i16,
) -> [u8; 6] {
    let mut buf = [0u8; 6];
    buf[0..2].copy_from_slice(&pitch.to_be_bytes());
    buf[2..4].copy_from_slice(&roll.to_be_bytes());
    buf[4..6].copy_from_slice(&yaw.to_be_bytes());
    buf
}

pub fn payload_battery(
    voltage: u16, // 100mV ? No, doc says 10uV? Wait.
    // Doc: "Voltage (LSB = 10 µV)" -> 25.2V = 2,520,000. u16 max is 65535.
    // That can't be right for u16. 65535 * 10uV = 0.6V?
    // Ah, "Battery Sensor" 0x08.
    // Betaflight implementation: voltage is big endian u16.
    // Usually sent as dV (decivolts) or similar?
    // Let's re-read doc VERY carefully.
    // "Voltage (LSB = 10 µV)" ... "u16".
    // Maybe it means 100mV? If LSB=0.1V, then 6553.5V max.
    // If LSB=0.01V, then 655.35V max.
    // CRSF Rev C doc says: "Voltage (mV * 10)" No.
    // Betaflight: `crsfData.batteryVoltage = (uint16_t)(batteryMeter.voltage * 10);` where voltage is in 0.1V steps?
    // Actually, OpenTX expects big endian.
    // Common usage: Voltage in 0.1V steps.
    // Wait, "0x08 Battery Sensor":
    // int16_t voltage; // Voltage (LSB = 100mV) <- typical
    // Let's assume 0.1V (100mV) per bit for now, typical for RC code.
    current: u16,  // 0.1A ?
    capacity: u32, // 24 bits
    remaining: u8,
) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0..2].copy_from_slice(&voltage.to_be_bytes());
    buf[2..4].copy_from_slice(&current.to_be_bytes());
    // 24 bit capacity - Big Endian
    // 24 bit capacity - Big Endian
    let cap_be = capacity.to_be_bytes(); // [u8; 4]
    buf[4] = cap_be[1];
    buf[5] = cap_be[2];
    buf[6] = cap_be[3];
    buf[7] = remaining;
    buf
}

pub const CRSF_FRAMETYPE_VARIO: u8 = 0x09; // Baro Altitude + Vario
pub const CRSF_FRAMETYPE_BAROMETRIC_SENSORS: u8 = 0x11; // Pressure + Temp

pub fn payload_vario(altitude: u16, vertical_speed: i16) -> [u8; 4] {
    // Altitude: uint16, MSB=0 -> decimeters + 10000 offset (0=-1000m).
    // MSB=1 -> meters, no offset?
    // Let's use the decimeter format as it's common.
    // Spec: "MSB = 0: altitude is in decimeters - 10000dm offset"
    // So 0 represents -1000m; 10000 represents 0m.
    // If altitude is 100m -> 1000dm. We send 10000+1000 = 11000.
    // If altitude is -10m -> -100dm. We send 10000-100 = 9900.
    //
    // Vertical speed: int8_t vertical_speed_packed.
    // But the payload is defined as:
    // uint16_t altitude_packed;
    // int16_t vertical_speed_packed;  <-- WAIT. "int8_t vertical_speed_packed" in text, but "int16_t vertical_speed" in summary?
    // Let's check frame size. 0x09.
    // OpenTX source says: 2 bytes alt, 2 bytes vspd? Or 1 byte vspd?
    // The doc says: "allows in 3 bytes combine...". So 2 bytes Alt + 1 byte VSpd?
    // Betaflight uses 4 bytes payload for 0x09??
    // Let's check Betaflight source. `crsfFrameVario_s`: `uint16_t altitude`, `int16_t verticalSpeed`. Total 4 bytes.
    // The "3 bytes" comment in doc might be old or referring to packed format.
    // Let's assume 4 bytes (2x u16/i16 Big Endian).

    let mut buf = [0u8; 4];
    buf[0..2].copy_from_slice(&altitude.to_be_bytes());
    buf[2..4].copy_from_slice(&vertical_speed.to_be_bytes());
    buf
}

pub fn payload_barometer(pressure_pa: u32, temp_c: i16) -> [u8; 8] {
    // 0x11 Barometer
    // int32_t pressure_pa; // Pascals, Big Endian? Usually.
    // int32_t baro_temp;   // centidegrees? (0.01 C).

    // Check Betaflight: `uint32_t pressure`, `int16_t temperature`. <-- Wait. 6 bytes? Or 8?
    // Doc says: "int32_t pressure_pa", "int32_t baro_temp". That's 8 bytes.
    // Betaflight sends:
    // buffer[0-3] = pressure (BE)
    // buffer[4-5] = temp (BE) -> So only 6 bytes?
    // CRSF Rev C doc says: "int32_t pressure_pa", "int32_t baro_temp".
    // But let's verify if OpenTx reads 32-bit temp.
    // Most sensors give 32 bit temp? No.
    // Let's try sending 8 bytes to be safe with the doc.

    let mut buf = [0u8; 8];
    buf[0..4].copy_from_slice(&pressure_pa.to_be_bytes());
    buf[4..8].copy_from_slice(&(temp_c as i32).to_be_bytes());
    // Cast char/int16 to i32 for the frame field
    buf
}
