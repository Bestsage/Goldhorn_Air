use core::str::FromStr;
use micromath::F32Ext;

#[derive(Debug, Clone, Copy, Default)]
pub struct GpsData {
    pub lat: f32,    // Decimal Degrees
    pub lon: f32,    // Decimal Degrees
    pub alt: f32,    // Meters (MSL)
    pub speed: f32,  // Knots
    pub course: f32, // Degrees
    pub sats: u8,
    pub active_ids: [u8; 12], // List of PRNs used in fix
    pub active_count: u8,
    pub fix: bool,
}

pub struct NmeaParser {
    buffer: heapless::String<128>,
    pub data: GpsData,
}

impl NmeaParser {
    pub fn new() -> Self {
        Self {
            buffer: heapless::String::new(),
            data: GpsData::default(),
        }
    }

    /// Process incoming bytes from UART
    pub fn push_data(&mut self, data: &[u8]) {
        for &b in data {
            if b == b'$' {
                self.buffer.clear();
            }

            if self.buffer.push(b as char).is_err() {
                // Buffer full, reset
                self.buffer.clear();
                continue;
            }

            if b == b'\n' {
                // End of sentence, parse it
                self.parse_sentence();
                self.buffer.clear();
            }
        }
    }

    fn parse_sentence(&mut self) {
        // Basic parser for GNGGA / GPGGA / GNRMC / GNGSA
        let s = self.buffer.as_str().trim();
        if !verify_checksum(s) {
            return;
        }

        if s.starts_with("$GNGGA") || s.starts_with("$GPGGA") {
            // $__GGA,time,lat,NS,lon,EW,qual,sats,hdop,alt,M,...
            let mut parts = s.split(',');

            // Skip ID
            parts.next();
            // Skip Time
            parts.next();

            // Latitude
            let lat_raw = parts.next().unwrap_or("");
            let ns = parts.next().unwrap_or("");

            // Longitude
            let lon_raw = parts.next().unwrap_or("");
            let ew = parts.next().unwrap_or("");

            // Fix Quality
            let qual = parts.next().unwrap_or("0");

            // Sats
            let sats = parts.next().unwrap_or("0");

            // HDOP
            parts.next();

            // Altitude
            let alt = parts.next().unwrap_or("0.0");

            // Update Data
            if let (Ok(l_val), Ok(o_val), Ok(a_val), Ok(s_val), Ok(q_val)) = (
                f32::from_str(lat_raw),
                f32::from_str(lon_raw),
                f32::from_str(alt),
                u8::from_str(sats),
                u8::from_str(qual),
            ) {
                // NMEA Lat/Lon is DDMM.MMMM format. Convert to Decimal Degrees.
                // e.g. 4807.038 => 48 deg + 07.038 min

                let lat_deg = (l_val / 100.0).floor();
                let lat_min = l_val - (lat_deg * 100.0);
                let mut latitude = lat_deg + (lat_min / 60.0);
                if ns == "S" {
                    latitude = -latitude;
                }

                let lon_deg = (o_val / 100.0).floor();
                let lon_min = o_val - (lon_deg * 100.0);
                let mut longitude = lon_deg + (lon_min / 60.0);
                if ew == "W" {
                    longitude = -longitude;
                }

                self.data.lat = latitude;
                self.data.lon = longitude;
                self.data.alt = a_val;
                self.data.sats = s_val;
                self.data.fix = q_val > 0;
            }
        } else if s.starts_with("$GNRMC") || s.starts_with("$GPRMC") {
            // $__RMC,time,status,lat,NS,lon,EW,speed,course,date,mag_var,stuff*CS
            let mut parts = s.split(',');
            // skip id, time, status, lat, ns, lon, ew
            for _ in 0..7 {
                parts.next();
            }

            let speed_raw = parts.next().unwrap_or("0.0");
            let course_raw = parts.next().unwrap_or("0.0");

            if let (Ok(spd), Ok(crs)) = (f32::from_str(speed_raw), f32::from_str(course_raw)) {
                self.data.speed = spd; // Knots
                self.data.course = crs;
            }
        } else if s.starts_with("$GNGSA") || s.starts_with("$GPGSA") {
            // $__GSA,mode1,mode2,id1,id2...id12,pdop,hdop,vdop*CS
            let mut parts = s.split(',');
            // Skip ID, Mode1, Mode2
            parts.next();
            parts.next();
            parts.next();

            let mut count = 0;
            let mut ids = [0u8; 12];

            for _ in 0..12 {
                if let Some(id_str) = parts.next() {
                    if let Ok(id) = u8::from_str(id_str) {
                        if id > 0 {
                            ids[count] = id;
                            count += 1;
                        }
                    }
                } else {
                    break;
                }
            }
            // Only update if we found some, or if we want to clear it (e.g. lost fix)
            // But GSA is sent regularly.
            self.data.active_ids = ids;
            self.data.active_count = count as u8;
        }
    }
}

fn verify_checksum(s: &str) -> bool {
    if let Some((content, check_str)) = s.split_once('*') {
        let content = content.strip_prefix('$').unwrap_or(content);
        let mut calc = 0u8;
        for b in content.bytes() {
            calc ^= b;
        }
        if let Ok(val) = u8::from_str_radix(check_str, 16) {
            return calc == val;
        }
    }
    false
}
