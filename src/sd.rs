//! SD-specific extensions to the core SDMMC protocol.

use aligned::{A4, Aligned};
use embedded_hal_async::delay::DelayNs;

pub use crate::common::*;
use crate::{
    Addressable, BlockCommand, BlockDevice, BlockReadCommand, BusAdapter, BusWidth, Command,
    ControlCommand, INIT_FREQ, MmcBus, MmcError, R1, R3, R6, R7, Signalling, common, sd,
};

/// Type marker for SD-specific extensions.
#[derive(Clone, Copy, Default)]
pub struct SD;

use core::{convert::TryInto, fmt, str};

// ============================================================================
// SD MEMORY COMMANDS
// ============================================================================

/// CMD3 — SEND_RELATIVE_ADDR (RCA)
pub struct Cmd3;
impl Command for Cmd3 {
    const INDEX: u8 = 3;
    type Resp<'a> = R6;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd3 {}

/// CMD3 — SEND_RELATIVE_ADDR (RCA)
pub fn send_relative_address() -> Cmd3 {
    Cmd3
}

/// CMD6 — SWITCH_FUNCTION
pub struct Cmd6<'a> {
    pub arg: u32,
    pub buf: &'a mut Aligned<A4, [u8; 64]>,
}
impl<'a> Command for Cmd6<'a> {
    const INDEX: u8 = 6;
    type Resp<'b>
        = R1
    where
        Self: 'b;
    fn arg(&self) -> u32 {
        self.arg
    }
}
impl<'a> BlockCommand for Cmd6<'a> {
    fn block_size(&self) -> u16 {
        64
    }

    fn block_count(&self) -> u32 {
        1
    }
}

impl<'a> BlockReadCommand for Cmd6<'a> {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        &mut *self.buf
    }
}

/// CMD6 — SWITCH_FUNCTION
pub fn cmd6(arg: u32, buf: &mut Aligned<A4, [u8; 64]>) -> Cmd6<'_> {
    Cmd6 { arg, buf }
}

/// CMD8 — SEND_IF_COND
pub struct Cmd8 {
    pub voltage: u8,
    pub checkpattern: u8,
}
impl Command for Cmd8 {
    const INDEX: u8 = 8;
    type Resp<'a> = R7;
    fn arg(&self) -> u32 {
        ((self.voltage as u32 & 0xF) << 8) | (self.checkpattern as u32)
    }
}
impl ControlCommand for Cmd8 {}

/// CMD8 — SEND_IF_COND
pub fn send_if_cond(voltage: u8, checkpattern: u8) -> Cmd8 {
    Cmd8 {
        voltage,
        checkpattern,
    }
}

/// CMD11 — VOLTAGE_SWITCH
pub struct Cmd11;
impl Command for Cmd11 {
    const INDEX: u8 = 11;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd11 {}

/// CMD11 — VOLTAGE_SWITCH
pub fn voltage_switch() -> Cmd11 {
    Cmd11
}

/// CMD19 — SEND_TUNING_BLOCK
pub struct Cmd19 {
    pub addr: u32,
}
impl Command for Cmd19 {
    const INDEX: u8 = 19;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.addr
    }
}
impl ControlCommand for Cmd19 {}

/// CMD19 — SEND_TUNING_BLOCK
pub fn send_tuning_block(addr: u32) -> Cmd19 {
    Cmd19 { addr }
}

/// CMD20 — SPEED_CLASS_CONTROL
pub struct Cmd20 {
    pub arg: u32,
}
impl Command for Cmd20 {
    const INDEX: u8 = 20;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.arg
    }
}
impl ControlCommand for Cmd20 {}

/// CMD20 — SPEED_CLASS_CONTROL
pub fn speed_class_control(arg: u32) -> Cmd20 {
    Cmd20 { arg }
}

/// CMD22 — ADDRESS_EXTENSION
pub struct Cmd22 {
    pub arg: u32,
}
impl Command for Cmd22 {
    const INDEX: u8 = 22;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.arg
    }
}
impl ControlCommand for Cmd22 {}

/// CMD22 — ADDRESS_EXTENSION
pub fn address_extension(arg: u32) -> Cmd22 {
    Cmd22 { arg }
}

/// CMD23 — SET_BLOCK_COUNT
pub struct Cmd23 {
    pub blockcount: u32,
}
impl Command for Cmd23 {
    const INDEX: u8 = 23;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.blockcount
    }
}
impl ControlCommand for Cmd23 {}

/// CMD23 — SET_BLOCK_COUNT
pub fn set_block_count(blockcount: u32) -> Cmd23 {
    Cmd23 { blockcount }
}

/// CMD32 — ERASE_WR_BLK_START_ADDR
pub struct Cmd32 {
    pub address: u32,
}
impl Command for Cmd32 {
    const INDEX: u8 = 32;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.address
    }
}
impl ControlCommand for Cmd32 {}

/// CMD32 — ERASE_WR_BLK_START_ADDR
pub fn erase_wr_blk_start_addr(address: u32) -> Cmd32 {
    Cmd32 { address }
}

/// CMD33 — ERASE_WR_BLK_END_ADDR
pub struct Cmd33 {
    pub address: u32,
}
impl Command for Cmd33 {
    const INDEX: u8 = 33;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.address
    }
}
impl ControlCommand for Cmd33 {}

/// CMD33 — ERASE_WR_BLK_END_ADDR
pub fn erase_wr_blk_end_addr(address: u32) -> Cmd33 {
    Cmd33 { address }
}

/// CMD36 — ERASE_GROUP_END
pub struct Cmd36 {
    pub address: u32,
}
impl Command for Cmd36 {
    const INDEX: u8 = 36;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.address
    }
}
impl ControlCommand for Cmd36 {}

/// CMD36: Sets the address of the last erase group within a continuous range to
/// be selected for erase
///
/// Address is either byte address or sector address (set in OCR)
pub fn erase_group_end(address: u32) -> Cmd36 {
    Cmd36 { address }
}

/// CMD58 — READ_OCR
pub struct Cmd58;
impl Command for Cmd58 {
    const INDEX: u8 = 58;
    type Resp<'a> = R3;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd58 {}

pub fn read_ocr() -> Cmd58 {
    Cmd58
}

// ============================================================================
// APPLICATION COMMANDS (ACMD) — treated as normal commands (Option A)
// ============================================================================

/// ACMD6 — SET_BUS_WIDTH
pub struct Acmd6 {
    pub bw4bit: bool,
}
impl Command for Acmd6 {
    const INDEX: u8 = 6;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        if self.bw4bit { 0b10 } else { 0b00 }
    }
}
impl ControlCommand for Acmd6 {}

/// ACMD6 — SET_BUS_WIDTH
pub fn set_bus_width(bw4bit: bool) -> Acmd6 {
    Acmd6 { bw4bit }
}

/// ACMD13 — SD_STATUS (block read, 64 bytes)
pub struct Acmd13<'a> {
    pub buf: &'a mut Aligned<A4, [u8; 64]>,
}
impl<'a> Command for Acmd13<'a> {
    const INDEX: u8 = 13;
    type Resp<'b>
        = R1
    where
        Self: 'b;
    fn arg(&self) -> u32 {
        0
    }
}
impl<'a> BlockCommand for Acmd13<'a> {
    fn block_size(&self) -> u16 {
        64
    }
    fn block_count(&self) -> u32 {
        1
    }
}
impl<'a> BlockReadCommand for Acmd13<'a> {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        &mut *self.buf
    }
}

/// ACMD13: SD Status
pub fn sd_status(status: &mut SDStatus) -> Acmd13<'_> {
    Acmd13 {
        buf: &mut status.inner,
    }
}

/// ACMD41 — SD_SEND_OP_COND
pub struct Acmd41 {
    pub host_high_capacity_support: bool,
    pub sdxc_power_control: bool,
    pub switch_to_1_8v_request: bool,
    pub voltage_window: u16,
}
impl Command for Acmd41 {
    const INDEX: u8 = 41;
    type Resp<'a> = R3;
    fn arg(&self) -> u32 {
        (u32::from(self.host_high_capacity_support) << 30)
            | (u32::from(self.sdxc_power_control) << 28)
            | (u32::from(self.switch_to_1_8v_request) << 24)
            | ((self.voltage_window as u32 & 0x1FF) << 15)
    }
}
impl ControlCommand for Acmd41 {}

/// ACMD41 — SD_SEND_OP_COND
pub fn sd_send_op_cond(
    host_high_capacity_support: bool,
    sdxc_power_control: bool,
    switch_to_1_8v_request: bool,
    voltage_window: u16,
) -> Acmd41 {
    Acmd41 {
        host_high_capacity_support,
        sdxc_power_control,
        switch_to_1_8v_request,
        voltage_window,
    }
}

/// ACMD51 — SEND_SCR (block read, 8 bytes)
pub struct Acmd51<'a> {
    pub inner: &'a mut Aligned<A4, [u8; 8]>,
}
impl<'a> Command for Acmd51<'a> {
    const INDEX: u8 = 51;
    type Resp<'b>
        = R1
    where
        Self: 'b;
    fn arg(&self) -> u32 {
        0
    }
}
impl<'a> BlockCommand for Acmd51<'a> {
    fn block_size(&self) -> u16 {
        8
    }
    fn block_count(&self) -> u32 {
        1
    }
}
impl<'a> BlockReadCommand for Acmd51<'a> {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        &mut *self.inner
    }
}

/// ACMD51: Reads the SCR
pub fn send_scr(scr: &mut SCR) -> Acmd51<'_> {
    Acmd51 { inner: &mut scr.0 }
}

#[non_exhaustive]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum SDSpecVersion {
    /// Version 1.0 and and 1.0.1
    V1_0,
    /// Version 1.10
    V1_10,
    /// Version 2.0
    V2,
    /// Version 3.0
    V3,
    /// Version 4.0
    V4,
    /// Version 5.0
    V5,
    /// Version 6.0
    V6,
    /// Version 7.0
    V7,
    /// Version not known by this crate
    Unknown,
}

/// SD CARD Configuration Register (SCR)
#[derive(Clone, Copy, Default)]
pub struct SCR(pub Aligned<A4, [u8; 8]>);

impl SCR {
    /// Create a new `SCR`
    #[inline]
    pub const fn new() -> Self {
        Self(Aligned([0u8; 8]))
    }

    fn inner_word(&self) -> u64 {
        u64::from_le_bytes(*self.0)
    }

    /// Physical Layer Specification Version Number
    pub fn version(&self) -> SDSpecVersion {
        let spec = (self.inner_word() >> 56) & 0xF;
        let spec3 = (self.inner_word() >> 47) & 1;
        let spec4 = (self.inner_word() >> 42) & 1;
        let specx = (self.inner_word() >> 38) & 0xF;

        // Ref PLSS_v7_10 Table 5-17
        match (spec, spec3, spec4, specx) {
            (0, 0, 0, 0) => SDSpecVersion::V1_0,
            (1, 0, 0, 0) => SDSpecVersion::V1_10,
            (2, 0, 0, 0) => SDSpecVersion::V2,
            (2, 1, 0, 0) => SDSpecVersion::V3,
            (2, 1, 1, 0) => SDSpecVersion::V4,
            (2, 1, _, 1) => SDSpecVersion::V5,
            (2, 1, _, 2) => SDSpecVersion::V6,
            (2, 1, _, 3) => SDSpecVersion::V7,
            _ => SDSpecVersion::Unknown,
        }
    }
    /// Bus widths supported
    pub fn bus_widths(&self) -> u8 {
        // Ref PLSS_v7_10 Table 5-21
        ((self.inner_word() >> 48) as u8) & 0xF
    }
    /// Supports 1-bit bus width
    pub fn bus_width_one(&self) -> bool {
        (self.inner_word() >> 48) & 1 != 0
    }
    /// Supports 4-bit bus width
    pub fn bus_width_four(&self) -> bool {
        (self.inner_word() >> 50) & 1 != 0
    }
}
impl core::fmt::Debug for SCR {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SCR: SD CARD Configuration Register")
            .field("Version", &self.version())
            .field("1-bit width", &self.bus_width_one())
            .field("4-bit width", &self.bus_width_four())
            .finish()
    }
}

impl OCR<SD> {
    /// VDD voltage window.
    // 00000000 00000000 00000000 00000000
    // 00000000 00000000 0
    //          11111111 1
    // OCR [23:15].
    pub fn voltage_window_mv(&self) -> Option<(u16, u16)> {
        let mut window = (self.0 >> 15) & 0x1FF;
        let mut min = 2_700;

        while window & 1 == 0 && window != 0 {
            min += 100;
            window >>= 1;
        }
        let mut max = min;
        while window != 0 {
            max += 100;
            window >>= 1;
        }

        if max == min { None } else { Some((min, max)) }
    }
    /// Switching to 1.8V Accepted (S18A). Only UHS-I cards support this bit
    // 00000000 00000000 00000000 00000000
    //        1
    // OCR [24].
    pub fn v18_allowed(&self) -> bool {
        self.0 & 0x0100_0000 != 0
    }
    /// Over 2TB support Status. Only SDUC card support this bit
    // 00000000 00000000 00000000 00000000
    //     1
    // OCR [27].
    pub fn over_2tb(&self) -> bool {
        self.0 & 0x0800_0000 != 0
    }
    /// Indicates whether the card supports UHS-II Interface
    // 00000000 00000000 00000000 00000000
    //   1
    // OCR [29].
    pub fn uhs2_card_status(&self) -> bool {
        self.0 & 0x2000_0000 != 0
    }
    /// Card Capacity Status (CCS)
    ///
    /// For SD cards, this is true for SDHC/SDXC/SDUC, false for SDSC
    pub fn high_capacity(&self) -> bool {
        self.0 & 0x4000_0000 != 0
    }
}
impl fmt::Debug for OCR<SD> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("OCR: Operation Conditions Register")
            .field(
                "Voltage Window (mV)",
                &self.voltage_window_mv().unwrap_or((0, 0)),
            )
            .field("S18A (UHS-I only)", &self.v18_allowed())
            .field("Over 2TB flag (SDUC only)", &self.over_2tb())
            .field("UHS-II Card", &self.uhs2_card_status())
            .field(
                "Card Capacity Status (CSS)",
                &if self.high_capacity() {
                    "SDHC/SDXC/SDUC"
                } else {
                    "SDSC"
                },
            )
            .field("Busy", &self.is_busy())
            .finish()
    }
}

impl CID<SD> {
    /// OEM/Application ID
    pub fn oem_id(&self) -> &str {
        str::from_utf8(&self.bytes[1..3]).unwrap_or(&"<ERR>")
    }
    /// Product name
    pub fn product_name(&self) -> &str {
        str::from_utf8(&self.bytes[3..8]).unwrap_or(&"<ERR>")
    }
    /// Product revision
    pub fn product_revision(&self) -> u8 {
        self.bytes[8]
    }
    /// Product serial number
    pub fn serial(&self) -> u32 {
        (self.inner() >> 24) as u32
    }
    /// Manufacturing date
    pub fn manufacturing_date(&self) -> (u8, u16) {
        (
            (self.inner() >> 8) as u8 & 0xF,             // Month
            ((self.inner() >> 12) as u16 & 0xFF) + 2000, // Year
        )
    }
}

impl fmt::Debug for CID<SD> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CID: Card Identification")
            .field("Manufacturer ID", &self.manufacturer_id())
            .field("OEM ID", &self.oem_id())
            .field("Product Name", &self.product_name())
            .field("Product Revision", &self.product_revision())
            .field("Product Serial Number", &self.serial())
            .field("Manufacturing Date", &self.manufacturing_date())
            .finish()
    }
}

impl CSD<SD> {
    /// Number of blocks in the card
    pub fn block_count(&self) -> u64 {
        match self.version() {
            0 => {
                // SDSC
                let c_size: u16 = ((self.0 >> 62) as u16) & 0xFFF;
                let c_size_mult: u8 = ((self.0 >> 47) as u8) & 7;

                ((c_size + 1) as u64) * ((1 << (c_size_mult + 2)) as u64)
            }
            1 => {
                // SDHC/SDXC
                (((self.0 >> 48) as u64 & 0x3F_FFFF) + 1) * 1024
            }
            2 => {
                // SDUC
                (((self.0 >> 48) as u64 & 0xFFF_FFFF) + 1) * 1024
            }
            _ => 0,
        }
    }
    /// Card size in bytes
    pub fn card_size(&self) -> u64 {
        let block_size_bytes = 1 << self.block_length() as u64;

        self.block_count() * block_size_bytes
    }
    /// Erase size (in blocks)
    pub fn erase_size_blocks(&self) -> u32 {
        if (self.0 >> 46) & 1 == 1 {
            // ERASE_BLK_EN
            1
        } else {
            let sector_size_tens = (self.0 >> 43) & 0x7;
            let sector_size_units = (self.0 >> 39) & 0xF;

            (sector_size_tens as u32 * 10) + (sector_size_units as u32)
        }
    }
}

impl fmt::Debug for CSD<SD> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CSD: Card Specific Data")
            .field("Transfer Rate", &self.transfer_rate())
            .field("Block Count", &self.block_count())
            .field("Card Size (bytes)", &self.card_size())
            .field("Read I (@min VDD)", &self.read_current_minimum_vdd())
            .field("Write I (@min VDD)", &self.write_current_minimum_vdd())
            .field("Read I (@max VDD)", &self.read_current_maximum_vdd())
            .field("Write I (@max VDD)", &self.write_current_maximum_vdd())
            .field("Erase Size (Blocks)", &self.erase_size_blocks())
            .finish()
    }
}

impl CardStatus<SD> {
    /// Command was executed without internal ECC
    pub fn ecc_disabled(&self) -> bool {
        self.0 & 0x4000 != 0
    }
    /// Extension function specific status
    pub fn fx_event(&self) -> bool {
        self.0 & 0x40 != 0
    }
    /// Authentication sequence error
    pub fn ake_seq_error(&self) -> bool {
        self.0 & 0x8 != 0
    }
}

impl fmt::Debug for CardStatus<SD> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Card Status")
            .field("Out of range error", &self.out_of_range())
            .field("Address error", &self.address_error())
            .field("Block len error", &self.block_len_error())
            .field("Erase seq error", &self.erase_seq_error())
            .field("Erase param error", &self.erase_param())
            .field("Write protect error", &self.wp_violation())
            .field("Card locked", &self.card_is_locked())
            .field("Password lock unlock error", &self.lock_unlock_failed())
            .field(
                "Crc check for the previous command failed",
                &self.com_crc_error(),
            )
            .field("Illegal command", &self.illegal_command())
            .field("Card internal ecc failed", &self.card_ecc_failed())
            .field("Internal card controller error", &self.cc_error())
            .field("General Error", &self.error())
            .field("Csd error", &self.csd_overwrite())
            .field("Write protect error", &self.wp_erase_skip())
            .field("Command ecc disabled", &self.ecc_disabled())
            .field("Erase sequence cleared", &self.erase_reset())
            .field("Card state", &self.state())
            .field("Buffer empty", &self.ready_for_data())
            .field("Extension event", &self.fx_event())
            .field("Card expects app cmd", &self.app_cmd())
            .field("Auth process error", &self.ake_seq_error())
            .finish()
    }
}

/// SD Status
///
/// Status bits related to SD Memory Card proprietary features
///
/// Ref PLSS_v7_10 Section 4.10.2 SD Status
#[derive(Clone, Copy)]
pub struct SDStatus {
    inner: Aligned<A4, [u8; 64]>,
}

impl Default for SDStatus {
    fn default() -> Self {
        Self::new()
    }
}

impl SDStatus {
    /// Create a new `SDStatus`
    #[inline]
    pub const fn new() -> Self {
        Self {
            inner: Aligned([0u8; 64]),
        }
    }

    fn inner_word(&self, i: usize) -> u32 {
        u32::from_le_bytes(self.inner[i * 4..i * 4 + 4].try_into().unwrap())
    }

    /// Current data bus width
    pub fn bus_width(&self) -> Option<BusWidth> {
        match (self.inner_word(15) >> 30) & 3 {
            0 => Some(BusWidth::W1),
            2 => Some(BusWidth::W4),
            _ => None,
        }
    }
    /// Is the card currently in the secured mode
    pub fn secure_mode(&self) -> bool {
        self.inner_word(15) & 0x2000_0000 != 0
    }
    /// SD Memory Card type (ROM, OTP, etc)
    pub fn sd_memory_card_type(&self) -> u16 {
        self.inner_word(15) as u16
    }
    /// SDHC / SDXC: Capacity of Protected Area in bytes
    pub fn protected_area_size(&self) -> u32 {
        self.inner_word(14)
    }
    /// Speed Class
    pub fn speed_class(&self) -> u8 {
        (self.inner_word(13) >> 24) as u8
    }
    /// "Performance Move" indicator in 1 MB/s units
    pub fn move_performance(&self) -> u8 {
        (self.inner_word(13) >> 16) as u8
    }
    /// Allocation Unit (AU) size. Lookup in PLSS v7_10 Table 4-47
    pub fn allocation_unit_size(&self) -> u8 {
        (self.inner_word(13) >> 12) as u8 & 0xF
    }
    /// Indicates N_Erase, in units of AU
    pub fn erase_size(&self) -> u16 {
        (self.inner_word(13) & 0xFF) as u16 | ((self.inner_word(12) >> 24) & 0xFF) as u16
    }
    /// Indicates T_Erase / Erase Timeout (s)
    pub fn erase_timeout(&self) -> u8 {
        (self.inner_word(12) >> 18) as u8 & 0x3F
    }
    /// Video speed class
    pub fn video_speed_class(&self) -> u8 {
        (self.inner_word(11) & 0xFF) as u8
    }
    /// Application Performance Class
    pub fn app_perf_class(&self) -> u8 {
        (self.inner_word(9) >> 16) as u8 & 0xF
    }
    /// Discard Support
    pub fn discard_support(&self) -> bool {
        self.inner_word(8) & 0x0200_0000 != 0
    }
}
impl fmt::Debug for SDStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SD Status")
            .field("Bus Width", &self.bus_width())
            .field("Secured Mode", &self.secure_mode())
            .field("SD Memory Card Type", &self.sd_memory_card_type())
            .field("Protected Area Size (B)", &self.protected_area_size())
            .field("Speed Class", &self.speed_class())
            .field("Video Speed Class", &self.video_speed_class())
            .field("Application Performance Class", &self.app_perf_class())
            .field("Move Performance (MB/s)", &self.move_performance())
            .field("AU Size", &self.allocation_unit_size())
            .field("Erase Size (units of AU)", &self.erase_size())
            .field("Erase Timeout (s)", &self.erase_timeout())
            .field("Discard Support", &self.discard_support())
            .finish()
    }
}

/// Card interface condition (R7)
pub type CIC = R7;

impl RCA<SD> {
    /// Status
    pub fn status(&self) -> u16 {
        self.0 as u16
    }
}

#[derive(Clone, Copy, Debug, Default)]
/// SD Card
pub struct Card {
    /// The type of this card
    pub card_type: CardCapacity,
    /// Operation Conditions Register
    pub ocr: OCR<SD>,
    /// Relative Card Address
    pub rca: u16,
    /// Card ID
    pub cid: CID<SD>,
    /// Card Specific Data
    pub csd: CSD<SD>,
    /// SD CARD Configuration Register
    pub scr: SCR,
    /// SD Status
    pub status: SDStatus,
}

impl Addressable for Card {
    type Ext = SD;

    /// Is this a standard or high capacity peripheral?
    fn get_capacity(&self) -> CardCapacity {
        self.card_type
    }

    /// Size in bytes
    fn size(&self) -> u64 {
        u64::from(self.csd.block_count()) * 512
    }

    fn supports_cmd23(&self) -> bool {
        // SCR.CMD_SUPPORT[1] per PLSS Table 5-21. CMD_SUPPORT lives at
        // SCR bits [35:32]; bit 1 = Set Block Count.
        (self.scr.inner_word() >> 33) & 1 != 0
    }
}

/// Card Storage Device
impl<B: MmcBus, D: DelayNs, const BLOCK_SIZE: usize> BlockDevice<Card, B, D, BLOCK_SIZE> {
    /// Create a new SD card
    pub async fn new_sd_card(bus: B, freq: u32, delay: D) -> Result<Self, MmcError> {
        let mut s = Self {
            info: Card::default(),
            bus: BusAdapter { bus, delay, rca: 0 },
        };

        s.acquire(freq).await?;

        Ok(s)
    }

    /// Initializes the card into a known state (or at least tries to).
    async fn acquire(&mut self, freq: u32) -> Result<(), MmcError> {
        // Clamp the frequency to the supported bus frequency.
        let freq = freq.clamp(0, self.bus.bus.supports_frequency());

        // Get the bus width configured in the Sdmmc peripheral
        let configured_bus_width = match self.bus.bus.supports_bus_width() {
            BusWidth::W8 => return Err(MmcError::Unsupported),
            bus_width => bus_width,
        };

        // While the SD/SDIO card or eMMC is in identification mode,
        // the SDMMC_CK frequency must be no more than 400 kHz.
        self.bus.bus.init_idle(INIT_FREQ).await?;

        self.bus.send_command(common::idle(), false).await?;

        // Check if cards supports CMD8 (with pattern)
        let cic: CIC = self
            .bus
            .send_command(send_if_cond(1, 0xAA), false)
            .await?
            .into();

        if cic.check_pattern != 0xAA {
            return Err(MmcError::Unsupported);
        }

        if cic.voltage & 1 == 0 {
            return Err(MmcError::Unsupported);
        }

        // Only request the 1.8V switch (S18A bit on ACMD41) when the
        // host actually has a level-shifter pin to drive — otherwise
        // a UHS-capable card may enter a partly-switched state when
        // we never follow up with CMD11. v1/v2 builds always read
        // `false` here (`has_vswitch` is hard-coded false).
        let request_18v = self.bus.bus.supports_1v8();

        // Note: this is a rather simplistic timeout loop. It can be improved later.
        let mut i = 0;
        self.info.ocr = loop {
            // 3.2-3.3V
            let voltage_window = 1 << 5;
            // Initialize card

            let ocr: OCR<SD> = self
                .bus
                .send_command(
                    sd_send_op_cond(true, false, request_18v, voltage_window),
                    true,
                )
                .await?
                .into();

            if !ocr.is_busy() {
                // Power up done
                break ocr;
            } else if i > 750 {
                return Err(MmcError::Timeout);
            }

            self.bus.delay.delay_ms(1).await;
            i += 1;
        };

        self.info.card_type = if self.info.ocr.high_capacity() {
            // Card is SDHC or SDXC or SDUC
            CardCapacity::HighCapacity
        } else {
            CardCapacity::StandardCapacity
        };

        if !self.bus.bus.supports_mmc() {
            // SPI mode
            self.info.ocr = self.bus.send_command(read_ocr(), false).await?.into();

            //
            // Switch to requested frequency
            //
            self.bus.bus.set_bus(BusWidth::W1, freq).await?;

            return Ok(());
        }

        // UHS-I voltage switch. Per SD Physical Layer Spec §3.7.5 the
        // voltage switch must happen here — between ACMD41 (which
        // moved the card to "ready" state) and CMD2 (which moves it
        // to "identification" state). CMD11 is only honoured by the
        // card while it's in the ready state. Doing the switch later
        // (e.g. after CMD2/CMD3) makes CMD11 time out.
        //
        // Only attempt if BOTH the host has a level-shifter pin AND
        // the card accepted the S18A request (ocr.v18_allowed()). On
        // failure, fall through to 3.3V HS — `voltage_switch()`
        // already restored peripheral + GPIO state.
        if request_18v && self.info.ocr.v18_allowed() {
            self.bus.send_command(voltage_switch(), false).await?;
        }

        self.info.cid = self
            .bus
            .send_command(common::all_send_cid(), false)
            .await?
            .into();

        self.bus.rca = RCA::<SD>::from(
            self.bus
                .send_command(send_relative_address(), false)
                .await?,
        )
        .address();

        self.info.csd = self
            .bus
            .send_command(common::send_csd(self.bus.rca), false)
            .await?
            .into();

        // Select card
        self.bus.select_card(Some(self.bus.rca)).await?;
        // Read SCR
        self.bus
            .read_blocks(sd::send_scr(&mut self.info.scr), true)
            .await?;

        // Select bus width based on Sdmmc configuration and card capability
        // Use 4-bit only if both the peripheral is configured for it AND the card supports it
        let (bus_width, acmd_arg) = match configured_bus_width {
            BusWidth::W4 if self.info.scr.bus_width_four() => (BusWidth::W4, 2),
            _ => (BusWidth::W4, 0),
        };

        self.bus
            .send_command(set_bus_width(acmd_arg == 2), true)
            .await?;

        // Up to 25Mhz
        self.bus
            .bus
            .set_bus(bus_width, freq.clamp(0, 25_000_000))
            .await?;

        // Read status
        self.bus
            .read_blocks(sd::sd_status(&mut self.info.status), true)
            .await?;

        if freq > 25_000_000 {
            // SDR104 needs DLYB tap tuning; SDR50 also accepts CKIN feedback. Below 50 MHz we cap at SDR25/HS.
            let request = if freq > 100_000_000 {
                Signalling::SDR104
            } else if freq > 50_000_000 {
                Signalling::SDR50
            } else {
                Signalling::SDR25
            };

            if request == self.switch_signalling_mode(request).await? {
                // Up to max_f
                self.bus.bus.set_bus(bus_width, freq).await?;

                let status: CardStatus<SD> = self
                    .bus
                    .send_command(common::card_status(self.info.rca, false), false)
                    .await?
                    .into();

                if status.state() != CurrentState::Transfer {
                    return Err(MmcError::SignalingSwitchFailed);
                }
            }

            // Read status after signalling change
            self.bus
                .read_blocks(sd::sd_status(&mut self.info.status), true)
                .await?;
        }

        Ok(())
    }

    /// Switch mode using CMD6.
    ///
    /// Attempt to set a new signalling mode. The selected
    /// signalling mode is returned. Expects the current clock
    /// frequency to be > 12.5MHz.
    ///
    /// SD only.
    async fn switch_signalling_mode(
        &mut self,
        signalling: Signalling,
    ) -> Result<Signalling, MmcError> {
        // NB PLSS v7_10 4.3.10.4: "the use of SET_BLK_LEN command is not
        // necessary"

        let set_function = 0x8000_0000
            | match signalling {
                // See PLSS v7_10 Table 4-11
                Signalling::DDR50 => 0xFF_FF04,
                Signalling::SDR104 => 0xFF_1F03,
                Signalling::SDR50 => 0xFF_1F02,
                Signalling::SDR25 => 0xFF_FF01,
                Signalling::SDR12 => 0xFF_FF00,
            };

        let mut buf = Aligned([0u8; 64]);

        self.bus
            .read_blocks(cmd6(set_function, &mut buf), false)
            .await?;

        // Host is allowed to use the new functions at least 8
        // clocks after the end of the switch command
        // transaction. We know the current clock period is < 80ns,
        // so a total delay of 640ns is required here
        self.bus.delay.delay_ns(640).await;

        // Function Selection of Function Group 1
        let selection =
            (u32::from_be(u32::from_le_bytes(buf[16..16 + 4].try_into().unwrap())) >> 24) & 0xF;

        match selection {
            0 => Ok(Signalling::SDR12),
            1 => Ok(Signalling::SDR25),
            2 => Ok(Signalling::SDR50),
            3 => Ok(Signalling::SDR104),
            4 => Ok(Signalling::DDR50),
            _ => Err(MmcError::Unsupported),
        }
    }
}
