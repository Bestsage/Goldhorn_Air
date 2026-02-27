use core::str::FromStr;
use micromath::F32Ext;

// ─── UBX Protocol Constants for u-blox M10 (CFG-VALSET keys) ───
// Key encoding: top byte = type (0x10=L/bool, 0x20=U1/E1, 0x30=U2, 0x50=X8)

// CFG-SIGNAL-* : GNSS signal enable
const CFG_SIGNAL_GPS_ENA: u32       = 0x1031001F;
const CFG_SIGNAL_GPS_L1CA_ENA: u32  = 0x10310001;
const CFG_SIGNAL_GAL_ENA: u32       = 0x10310021;
const CFG_SIGNAL_GAL_E1_ENA: u32    = 0x10310007;
const CFG_SIGNAL_BDS_ENA: u32       = 0x10310022;
const CFG_SIGNAL_BDS_B1_ENA: u32    = 0x1031000D;
const CFG_SIGNAL_GLO_ENA: u32       = 0x10310025;
const CFG_SIGNAL_GLO_L1_ENA: u32    = 0x10310018;
const CFG_SIGNAL_SBAS_ENA: u32      = 0x10310020;
const CFG_SIGNAL_SBAS_L1CA_ENA: u32 = 0x10310005;
const CFG_SIGNAL_QZSS_ENA: u32      = 0x10310024;
const CFG_SIGNAL_QZSS_L1CA_ENA: u32 = 0x10310012;

// CFG-SBAS-*
const CFG_SBAS_USE_TESTMODE: u32  = 0x10360002;
const CFG_SBAS_USE_RANGING: u32   = 0x10360003;
const CFG_SBAS_USE_DIFFCORR: u32  = 0x10360004;
const CFG_SBAS_PRNSCANMASK: u32   = 0x50360006;

// CFG-NAVSPG-*
const CFG_NAVSPG_DYNMODEL: u32    = 0x20110021;

// CFG-RATE-*
const CFG_RATE_MEAS: u32          = 0x30210001;
const CFG_RATE_NAV: u32           = 0x30210002;
const CFG_RATE_TIMEREF: u32       = 0x20210003;

// CFG-MSGOUT-NMEA (UART1)
const CFG_MSGOUT_GLL_UART1: u32   = 0x209100CA;
const CFG_MSGOUT_VTG_UART1: u32   = 0x209100B1;
const CFG_MSGOUT_GGA_UART1: u32   = 0x209100BB;
const CFG_MSGOUT_GSA_UART1: u32   = 0x209100BF;
const CFG_MSGOUT_GSV_UART1: u32   = 0x209100C4;
const CFG_MSGOUT_RMC_UART1: u32   = 0x209100AC;

// Dynamic model values
const DYNMODEL_AIRBORNE_4G: u8 = 8;

// EGNOS PRN scanmask: PRN 123(bit3), 126(bit6), 136(bit16)
const EGNOS_SCANMASK: u64 = (1 << 3) | (1 << 6) | (1 << 16); // 0x0001_0048

// ─── GNSS System ID (for constellation tracking in GSV) ───
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GnssSystem {
    Unknown = 0,
    Gps     = 1,
    Glonass = 2,
    Galileo = 3,
    Beidou  = 4,
    Sbas    = 5,
    Qzss    = 6,
}

impl Default for GnssSystem {
    fn default() -> Self { GnssSystem::Unknown }
}

impl GnssSystem {
    /// Single-char prefix for compact SV display
    pub fn prefix(self) -> char {
        match self {
            GnssSystem::Gps     => 'G',
            GnssSystem::Glonass => 'R',
            GnssSystem::Galileo => 'E',
            GnssSystem::Beidou  => 'C',
            GnssSystem::Sbas    => 'S',
            GnssSystem::Qzss    => 'Q',
            GnssSystem::Unknown => '?',
        }
    }
}

// ─── UBX Message Builder ───
pub struct UbxBuilder {
    pub buf: [u8; 128],
    pub idx: usize,
}

impl UbxBuilder {
    fn new() -> Self {
        let mut s = Self { buf: [0u8; 128], idx: 10 };
        s.buf[0] = 0xB5; // sync1
        s.buf[1] = 0x62; // sync2
        s.buf[2] = 0x06; // class: CFG
        s.buf[3] = 0x8A; // id: VALSET
        s.buf[6] = 0x00; // version (transactionless)
        s.buf[7] = 0x01; // layers: RAM only
        s
    }

    fn add_key(&mut self, key: u32) {
        let i = self.idx;
        self.buf[i]   = (key & 0xFF) as u8;
        self.buf[i+1] = ((key >> 8) & 0xFF) as u8;
        self.buf[i+2] = ((key >> 16) & 0xFF) as u8;
        self.buf[i+3] = ((key >> 24) & 0xFF) as u8;
        self.idx += 4;
    }

    fn add_bool(&mut self, key: u32, val: bool) {
        self.add_key(key);
        self.buf[self.idx] = val as u8;
        self.idx += 1;
    }

    fn add_u8(&mut self, key: u32, val: u8) {
        self.add_key(key);
        self.buf[self.idx] = val;
        self.idx += 1;
    }

    fn add_u16(&mut self, key: u32, val: u16) {
        self.add_key(key);
        self.buf[self.idx] = (val & 0xFF) as u8;
        self.buf[self.idx+1] = ((val >> 8) & 0xFF) as u8;
        self.idx += 2;
    }

    fn add_u64(&mut self, key: u32, val: u64) {
        self.add_key(key);
        for i in 0..8 {
            self.buf[self.idx + i] = ((val >> (i * 8)) & 0xFF) as u8;
        }
        self.idx += 8;
    }

    fn finalize(&mut self) -> usize {
        let payload_len = (self.idx - 6) as u16;
        self.buf[4] = (payload_len & 0xFF) as u8;
        self.buf[5] = ((payload_len >> 8) & 0xFF) as u8;
        let (ck_a, ck_b) = ubx_checksum(&self.buf[2..self.idx]);
        self.buf[self.idx] = ck_a;
        self.buf[self.idx + 1] = ck_b;
        self.idx += 2;
        self.idx
    }
}

fn ubx_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in data {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

// ─── Public UBX config message builders ───

/// Message 1: Enable GNSS constellations useful in Europe
/// GPS + Galileo + BeiDou + GLONASS + SBAS(EGNOS)
/// QZSS disabled: Japan-only system, wastes M10 tracking channels
/// with weak 11-15 dB-Hz signals that will never contribute to a fix.
pub fn ubx_cfg_gnss_all() -> ([u8; 128], usize) {
    let mut b = UbxBuilder::new();
    // GPS — primary constellation, worldwide
    b.add_bool(CFG_SIGNAL_GPS_ENA, true);
    b.add_bool(CFG_SIGNAL_GPS_L1CA_ENA, true);
    // Galileo — European constellation, best geometry in EU
    b.add_bool(CFG_SIGNAL_GAL_ENA, true);
    b.add_bool(CFG_SIGNAL_GAL_E1_ENA, true);
    // BeiDou — good EU coverage, strong signals (28-30 dB-Hz observed)
    b.add_bool(CFG_SIGNAL_BDS_ENA, true);
    b.add_bool(CFG_SIGNAL_BDS_B1_ENA, true);
    // GLONASS — good EU coverage, adds geometric diversity
    b.add_bool(CFG_SIGNAL_GLO_ENA, true);
    b.add_bool(CFG_SIGNAL_GLO_L1_ENA, true);
    // SBAS (EGNOS) — European augmentation, improves accuracy
    b.add_bool(CFG_SIGNAL_SBAS_ENA, true);
    b.add_bool(CFG_SIGNAL_SBAS_L1CA_ENA, true);
    // QZSS — DISABLED (Japan/Oceania only, useless in Europe)
    b.add_bool(CFG_SIGNAL_QZSS_ENA, false);
    b.add_bool(CFG_SIGNAL_QZSS_L1CA_ENA, false);
    let len = b.finalize();
    (b.buf, len)
}

/// Message 2: Airborne 4G + SBAS EGNOS + 10Hz rate + disable GLL/VTG
pub fn ubx_cfg_nav_sbas_rate() -> ([u8; 128], usize) {
    let mut b = UbxBuilder::new();
    // Dynamic model: Airborne 4G (for drones)
    b.add_u8(CFG_NAVSPG_DYNMODEL, DYNMODEL_AIRBORNE_4G);
    // SBAS: enable ranging + diffcorr, disable testmode
    b.add_bool(CFG_SBAS_USE_TESTMODE, false);
    b.add_bool(CFG_SBAS_USE_RANGING, true);
    b.add_bool(CFG_SBAS_USE_DIFFCORR, true);
    // EGNOS PRN scan mask
    b.add_u64(CFG_SBAS_PRNSCANMASK, EGNOS_SCANMASK);
    // 10Hz update rate (100ms measurement period)
    b.add_u16(CFG_RATE_MEAS, 100);
    b.add_u16(CFG_RATE_NAV, 1);
    b.add_u8(CFG_RATE_TIMEREF, 0); // UTC
    // Disable useless NMEA sentences
    b.add_u8(CFG_MSGOUT_GLL_UART1, 0);
    b.add_u8(CFG_MSGOUT_VTG_UART1, 0);
    // GGA + RMC at full 10Hz rate (critical for nav)
    b.add_u8(CFG_MSGOUT_GGA_UART1, 1);
    b.add_u8(CFG_MSGOUT_RMC_UART1, 1);
    // GSA at 2Hz (every 5th nav cycle) — DOP doesn't change fast
    b.add_u8(CFG_MSGOUT_GSA_UART1, 5);
    // GSV at 1Hz (every 10th nav cycle) — saves ~10KB/s bandwidth!
    b.add_u8(CFG_MSGOUT_GSV_UART1, 10);
    let len = b.finalize();
    (b.buf, len)
}

// ─── GPS State Machine (inspired by Betaflight gps.c) ───
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpsState {
    Unknown,
    DetectBaud,
    Initialised,
    ReceivingData,
    LostCommunication,
}

impl Default for GpsState {
    fn default() -> Self {
        GpsState::Unknown
    }
}

// Per-satellite info (like Betaflight GPS_svinfo)
#[derive(Debug, Clone, Copy, Default)]
pub struct SvInfo {
    pub svid: u8,       // PRN / satellite ID
    pub cno: u8,        // C/N₀ (dB-Hz, 0-99)
    pub gnss: GnssSystem, // which constellation
}

pub const GPS_SV_MAXSATS: usize = 48;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NmeaFrame {
    None,
    Gga,
    Rmc,
    Gsv,
    Gsa,
    Vtg,
    Gll,
}

impl Default for NmeaFrame {
    fn default() -> Self {
        NmeaFrame::None
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GpsData {
    // ── Position / Navigation ──
    pub lat: f32,          // Decimal Degrees
    pub lon: f32,          // Decimal Degrees
    pub alt: f32,          // Metres (MSL)
    pub speed: f32,        // Knots
    pub speed_cms: u32,    // cm/s  (Betaflight-style)
    pub course: f32,       // Degrees
    pub ground_course: u16, // Degrees × 10

    // ── Satellites ──
    pub sats: u8,
    pub sats_in_view: u8,
    pub fix_quality: u8,    // 0=no fix 1=GPS 2=DGPS …
    pub fix: bool,

    // ── DOP  (× 100, like Betaflight) ──
    pub hdop: f32,          // float for display
    pub hdop_i: u16,        // integer ×100
    pub pdop_i: u16,
    pub vdop_i: u16,

    // ── Active PRNs (from GSA) ──
    pub active_ids: [u8; 12],
    pub active_count: u8,

    // ── Diagnostics / counters (Betaflight-style) ──
    pub sentences_rx: u16,       // total valid sentences parsed
    pub checksum_errors: u16,    // checksum mismatches
    pub frame_errors: u16,       // buffer overflows / malformed frames
    pub gga_count: u16,          // GGA sentences received
    pub rmc_count: u16,          // RMC
    pub gsa_count: u16,          // GSA
    pub gsv_count: u16,          // GSV
    pub vtg_count: u16,          // VTG (known, not parsed)
    pub gll_count: u16,          // GLL (known, not parsed)
    pub unknown_count: u16,      // unrecognised sentence IDs
    pub last_frame: NmeaFrame,   // which sentence was last parsed

    // ── Timing  (caller fills these via update_timing) ──
    pub last_nav_msg_ms: u32,     // millis() at last nav solution (GGA)
    pub nav_interval_ms: u32,     // ms between last two nav solutions
    pub last_byte_ms: u32,        // millis() at last received byte

    // ── State machine ──
    pub state: GpsState,
    pub timeouts: u16,            // number of communication timeouts

    // ── UTC time & date from RMC ──
    pub utc_time: u32,            // hhmmss00  (Betaflight format)
    pub utc_date: u32,            // ddmmyy

    // ── Per-satellite table (from GSV) ──
    pub sv_count: u8,
    pub svinfo: [SvInfo; GPS_SV_MAXSATS],
    pub last_gsv_reset_ms: u32,   // millis() when sats_in_view was last reset
}

impl Default for GpsData {
    fn default() -> Self {
        // Manual impl because [SvInfo; 48] doesn't have auto Default
        // SAFETY: all-zero is valid for every field (f32=0.0, u8=0, bool=false, enums have 0-variant)
        unsafe { core::mem::zeroed() }
    }
}

/// Timeout before we declare lost communication (Betaflight: 2500 ms)
pub const GPS_TIMEOUT_MS: u32 = 2500;

pub struct NmeaParser {
    buffer: heapless::String<128>,
    pub data: GpsData,
    // internal GSV accumulator
    gsv_sv_index: u8,
}

impl NmeaParser {
    pub fn new() -> Self {
        Self {
            buffer: heapless::String::new(),
            data: GpsData::default(),
            gsv_sv_index: 0,
        }
    }

    /// Call this every loop iteration with current millis() and how many
    /// bytes were received this tick. Mirrors Betaflight's gpsUpdate().
    pub fn update_timing(&mut self, now_ms: u32, bytes_this_tick: usize) {
        if bytes_this_tick > 0 {
            self.data.last_byte_ms = now_ms;
        }

        match self.data.state {
            GpsState::Unknown => {
                if bytes_this_tick > 0 {
                    self.data.state = GpsState::DetectBaud;
                }
            }
            GpsState::DetectBaud => {
                // We received valid NMEA → move to Initialised
                if self.data.sentences_rx > 0 {
                    self.data.state = GpsState::Initialised;
                }
            }
            GpsState::Initialised => {
                // Got a GGA with fix → ReceivingData
                if self.data.gga_count > 0 {
                    self.data.state = GpsState::ReceivingData;
                }
            }
            GpsState::ReceivingData => {
                // Timeout based on last received BYTE (not just GGA) to avoid
                // false timeouts when GGA fails checksum but other data flows
                if now_ms.wrapping_sub(self.data.last_byte_ms) > GPS_TIMEOUT_MS {
                    self.data.state = GpsState::LostCommunication;
                    self.data.timeouts += 1;
                    // Don't reset fix/sats — position data may still be valid
                    // (Betaflight keeps last known position on timeout)
                }
            }
            GpsState::LostCommunication => {
                // Any new bytes → try again
                if bytes_this_tick > 0 {
                    self.data.state = GpsState::DetectBaud;
                }
            }
        }
    }

    /// Process incoming bytes from UART
    pub fn push_data(&mut self, data: &[u8]) {
        for &b in data {
            if b == b'$' {
                self.buffer.clear();
            }

            if self.buffer.push(b as char).is_err() {
                // Buffer overflow → count as frame error (Betaflight: gpsData.errors++)
                self.data.frame_errors = self.data.frame_errors.wrapping_add(1);
                self.buffer.clear();
                continue;
            }

            if b == b'\n' {
                self.parse_sentence();
                self.buffer.clear();
            }
        }
    }

    fn parse_sentence(&mut self) {
        // Copy the buffer to avoid borrow conflict (self.buffer vs &mut self)
        let mut local: heapless::String<128> = heapless::String::new();
        let _ = local.push_str(self.buffer.as_str().trim());
        let s = local.as_str();

        if s.len() < 6 {
            return; // too short to be valid
        }

        if !verify_checksum(s) {
            self.data.checksum_errors = self.data.checksum_errors.wrapping_add(1);
            return;
        }

        // Classify frame (Betaflight style: compare &string[2])
        let frame = if s.len() >= 6 {
            match &s[3..6] {
                "GGA" => NmeaFrame::Gga,
                "RMC" => NmeaFrame::Rmc,
                "GSA" => NmeaFrame::Gsa,
                "GSV" => NmeaFrame::Gsv,
                "VTG" => NmeaFrame::Vtg,
                "GLL" => NmeaFrame::Gll,
                _ => NmeaFrame::None,
            }
        } else {
            NmeaFrame::None
        };

        self.data.sentences_rx = self.data.sentences_rx.wrapping_add(1);
        self.data.last_frame = frame;

        match frame {
            NmeaFrame::Gga => self.parse_gga(s),
            NmeaFrame::Rmc => self.parse_rmc(s),
            NmeaFrame::Gsa => self.parse_gsa(s),
            NmeaFrame::Gsv => self.parse_gsv(s),
            NmeaFrame::Vtg => {
                self.data.vtg_count = self.data.vtg_count.wrapping_add(1);
            }
            NmeaFrame::Gll => {
                self.data.gll_count = self.data.gll_count.wrapping_add(1);
            }
            NmeaFrame::None => {
                self.data.unknown_count = self.data.unknown_count.wrapping_add(1);
            }
        }
    }

    // ────── GGA ──────
    fn parse_gga(&mut self, s: &str) {
        self.data.gga_count = self.data.gga_count.wrapping_add(1);
        // $xxGGA,time,lat,NS,lon,EW,qual,sats,hdop,alt,M,geoid,M,…*CS
        let mut parts = s.split(',');
        parts.next(); // ID

        // Time (field 1)
        let _time_str = parts.next().unwrap_or("");

        // Lat (field 2) + NS (field 3)
        let lat_raw = parts.next().unwrap_or("");
        let ns = parts.next().unwrap_or("");

        // Lon (field 4) + EW (field 5)
        let lon_raw = parts.next().unwrap_or("");
        let ew = parts.next().unwrap_or("");

        // Fix Quality (field 6)
        let qual_str = parts.next().unwrap_or("");

        // Sats (field 7)
        let sats_str = parts.next().unwrap_or("");

        // HDOP (field 8)
        let hdop_str = parts.next().unwrap_or("");

        // Alt (field 9)
        let alt_str = parts.next().unwrap_or("");

        // Parse fix quality
        if let Ok(q) = u8::from_str(qual_str) {
            self.data.fix_quality = q;
            self.data.fix = q > 0;
        } else {
            self.data.fix_quality = 0;
            self.data.fix = false;
        }

        if let Ok(s_val) = u8::from_str(sats_str) {
            self.data.sats = s_val;
        }

        if let Ok(h) = f32::from_str(hdop_str) {
            self.data.hdop = h;
            self.data.hdop_i = (h * 100.0) as u16;
        }

        if let Ok(a) = f32::from_str(alt_str) {
            self.data.alt = a;
        }

        // Lat/Lon (Betaflight GPS_coord_to_degrees style, but as f32)
        if let Ok(l_val) = f32::from_str(lat_raw) {
            let lat_deg = (l_val / 100.0).floor();
            let lat_min = l_val - (lat_deg * 100.0);
            let mut latitude = lat_deg + (lat_min / 60.0);
            if ns == "S" {
                latitude = -latitude;
            }
            self.data.lat = latitude;
        }

        if let Ok(o_val) = f32::from_str(lon_raw) {
            let lon_deg = (o_val / 100.0).floor();
            let lon_min = o_val - (lon_deg * 100.0);
            let mut longitude = lon_deg + (lon_min / 60.0);
            if ew == "W" {
                longitude = -longitude;
            }
            self.data.lon = longitude;
        }

        // Nav timing (Betaflight: gpsData.lastNavMessage/navIntervalMs)
        let prev = self.data.last_nav_msg_ms;
        // We don't have millis() here, caller must set last_byte_ms.
        // Use last_byte_ms as proxy for "now".
        let now = self.data.last_byte_ms;
        if prev > 0 {
            self.data.nav_interval_ms = now.wrapping_sub(prev);
        }
        self.data.last_nav_msg_ms = now;
    }

    // ────── RMC ──────
    fn parse_rmc(&mut self, s: &str) {
        self.data.rmc_count = self.data.rmc_count.wrapping_add(1);
        // $xxRMC,time,status,lat,NS,lon,EW,speed,course,date,…*CS
        let mut parts = s.split(',');
        parts.next(); // ID

        // Time (field 1) — hhmmss.ss
        let time_str = parts.next().unwrap_or("");
        if time_str.len() >= 6 {
            // Store as Betaflight: grab_fields(str,2)  →  hhmmsscc
            if let Ok(t) = u32::from_str(&time_str[..6]) {
                self.data.utc_time = t * 100; // hhmmss → hhmmss00
            }
        }

        // Status (field 2) — A=active, V=void
        let _status = parts.next().unwrap_or("");

        // Skip lat/NS/lon/EW (fields 3-6), we already get these from GGA
        for _ in 0..4 {
            parts.next();
        }

        // Speed (field 7) — knots
        let speed_raw = parts.next().unwrap_or("");
        // Course (field 8) — degrees
        let course_raw = parts.next().unwrap_or("");

        // Date (field 9) — ddmmyy
        let date_str = parts.next().unwrap_or("");
        if let Ok(d) = u32::from_str(date_str) {
            self.data.utc_date = d;
        }

        if let Ok(spd) = f32::from_str(speed_raw) {
            self.data.speed = spd; // Knots
            // cm/s: (knots × 5144) / 1000  (Betaflight formula)
            self.data.speed_cms = ((spd * 5144.0) / 1000.0) as u32;
        }
        if let Ok(crs) = f32::from_str(course_raw) {
            self.data.course = crs;
            self.data.ground_course = (crs * 10.0) as u16; // deg×10
        }
    }

    // ────── GSA ──────
    fn parse_gsa(&mut self, s: &str) {
        self.data.gsa_count = self.data.gsa_count.wrapping_add(1);
        // $xxGSA,mode1,mode2,id1…id12,pdop,hdop,vdop*CS
        let mut parts = s.split(',');
        parts.next(); // ID
        parts.next(); // Mode1
        parts.next(); // Mode2

        let mut count = 0usize;
        let mut ids = [0u8; 12];
        for _ in 0..12 {
            if let Some(id_str) = parts.next() {
                if let Ok(id) = u8::from_str(id_str) {
                    if id > 0 && count < 12 {
                        ids[count] = id;
                        count += 1;
                    }
                }
            } else {
                break;
            }
        }
        self.data.active_ids = ids;
        self.data.active_count = count as u8;

        // PDOP (field 15)
        if let Some(pdop_str) = parts.next() {
            if let Ok(p) = f32::from_str(pdop_str.split('*').next().unwrap_or(pdop_str)) {
                self.data.pdop_i = (p * 100.0) as u16;
            }
        }
        // HDOP (field 16)
        if let Some(hdop_str) = parts.next() {
            if let Ok(h) = f32::from_str(hdop_str.split('*').next().unwrap_or(hdop_str)) {
                self.data.hdop_i = (h * 100.0) as u16;
                self.data.hdop = h;
            }
        }
        // VDOP (field 17)
        if let Some(vdop_str) = parts.next() {
            if let Ok(v) = f32::from_str(vdop_str.split('*').next().unwrap_or(vdop_str)) {
                self.data.vdop_i = (v * 100.0) as u16;
            }
        }
    }

    // ────── GSV (per-satellite details, like Betaflight FRAME_GSV) ──────
    fn parse_gsv(&mut self, s: &str) {
        self.data.gsv_count = self.data.gsv_count.wrapping_add(1);
        // $xxGSV,totalMsgs,msgNum,satInView, [svid,elev,azim,cno] × 1-4, *CS
        let mut parts = s.split(',');
        let id_str = parts.next().unwrap_or(""); // ID ($GPGSV / $GAGSV …)
        let _total_msgs = parts.next().unwrap_or("");
        let msg_num_str = parts.next().unwrap_or("");
        let siv_str = parts.next().unwrap_or("");

        let msg_num: u8 = u8::from_str(msg_num_str).unwrap_or(0);

        // Identify constellation from NMEA talker ID ($GPgsv, $GLgsv, $GAgsv, $GBgsv, $GQgsv)
        let gnss = if id_str.len() >= 3 {
            match &id_str[1..3] {
                "GP" => GnssSystem::Gps,
                "GL" => GnssSystem::Glonass,
                "GA" => GnssSystem::Galileo,
                "GB" | "BD" => GnssSystem::Beidou,
                "GQ" | "QZ" => GnssSystem::Qzss,
                _ => GnssSystem::Unknown,
            }
        } else {
            GnssSystem::Unknown
        };

        // sats_in_view: reset when we see msg_num=1 AND more than
        // 500ms since last reset (= new GSV cycle). This avoids the
        // accumulation overflow when GPS GSV sentence is missed.
        if msg_num == 1 {
            if let Ok(n) = u8::from_str(siv_str) {
                let since_reset = self.data.last_byte_ms.wrapping_sub(self.data.last_gsv_reset_ms);
                if gnss == GnssSystem::Gps || since_reset > 500 {
                    // Start of new full cycle
                    self.data.sats_in_view = n;
                    self.gsv_sv_index = 0;
                    self.data.sv_count = 0;
                    self.data.last_gsv_reset_ms = self.data.last_byte_ms;
                } else {
                    self.data.sats_in_view = self.data.sats_in_view.saturating_add(n);
                }
            }
        }

        // Parse up-to 4 satellite records per GSV sentence
        // Fields repeat: svid, elev, azim, cno
        for _ in 0..4 {
            let svid_s = match parts.next() {
                Some(s) => s,
                None => break,
            };
            let _elev_s = parts.next().unwrap_or("");
            let _azim_s = parts.next().unwrap_or("");
            let cno_s_raw = parts.next().unwrap_or("");
            // cno field may contain *checksum on last satellite
            let cno_s = cno_s_raw.split('*').next().unwrap_or(cno_s_raw);

            let idx = self.gsv_sv_index as usize;
            if idx >= GPS_SV_MAXSATS {
                break;
            }

            let svid = u8::from_str(svid_s).unwrap_or(0);
            if svid == 0 {
                continue;
            }

            self.data.svinfo[idx] = SvInfo {
                svid,
                cno: u8::from_str(cno_s).unwrap_or(0),
                gnss,
            };
            self.gsv_sv_index += 1;
            self.data.sv_count = self.gsv_sv_index;
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
        // Only take first 2 hex chars (ignore trailing \r\n or garbage)
        let hex = if check_str.len() >= 2 { &check_str[..2] } else { check_str };
        if let Ok(val) = u8::from_str_radix(hex.trim(), 16) {
            return calc == val;
        }
    }
    false
}
