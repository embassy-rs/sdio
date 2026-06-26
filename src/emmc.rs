//! eMMC-specific extensions to the core SDMMC protocol.

use aligned::{A4, Aligned};
use embedded_hal_async::delay::DelayNs;

pub use crate::common::*;
use crate::{
    Acquirable, Addressable, BlockCommand, BlockDevice, BlockReadCommand, BusAdapter, BusWidth,
    Command, ControlCommand, MmcBus, MmcError, R1, R1b, R3, common,
};

use core::{convert::TryInto, fmt, marker::PhantomData, str};

// ============================================================================
// eMMC COMMANDS (MMC protocol)
// ============================================================================

/// CMD1 — SEND_OP_COND (MMC only)
/// Response: R3 (no CRC)
pub struct Cmd1 {
    pub ocr: u32,
}
impl Command for Cmd1 {
    const INDEX: u8 = 1;
    type Resp<'a> = R3;
    fn arg(&self) -> u32 {
        self.ocr
    }
}
impl ControlCommand for Cmd1 {}

/// CMD3 — ASSIGN_RELATIVE_ADDR (RCA)
pub struct Cmd3 {
    pub address: u16,
}
impl Command for Cmd3 {
    const INDEX: u8 = 3;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        (self.address as u32) << 16
    }
}
impl ControlCommand for Cmd3 {}

/// CMD3: Assigns relative address (RCA) to the Device
pub fn assign_relative_address(address: u16) -> Cmd3 {
    Cmd3 { address }
}

/// CMD1: Ask all cards to send their supported OCR, or become inactive if they cannot be
/// supported.
pub fn send_op_cond(ocr: u32) -> Cmd1 {
    Cmd1 { ocr }
}

/// CMD5 — SLEEP / AWAKE (MMC version)
/// NOTE: This is *not* SDIO CMD5.
pub struct Cmd5 {
    pub sleep: bool,
    pub rca: u16,
}
impl Command for Cmd5 {
    const INDEX: u8 = 5;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        ((self.sleep as u32) << 15) | ((self.rca as u32) << 16)
    }
}
impl ControlCommand for Cmd5 {}

/// CMD6 — SWITCH (MMC version)
/// Used to write EXT_CSD fields.
pub struct Cmd6 {
    pub access: u8,  // 0 = command set, 1 = set bits, 2 = clear bits, 3 = write byte
    pub index: u8,   // EXT_CSD index
    pub value: u8,   // value to write
    pub cmd_set: u8, // usually 0
}
impl Command for Cmd6 {
    const INDEX: u8 = 6;
    type Resp<'a> = R1b; // MMC SWITCH returns R1b (busy)
    fn arg(&self) -> u32 {
        ((self.access as u32) << 24)
            | ((self.index as u32) << 16)
            | ((self.value as u32) << 8)
            | (self.cmd_set as u32)
    }
}
impl ControlCommand for Cmd6 {}

/// Specifies a method of modifying a field of EXT_CSD. Used for CMD6.
pub enum AccessMode {
    // The 0b00 pattern corresponds to Command Set, which has different semantics.
    SetBits = 0b01,
    ClearBits = 0b10,
    WriteByte = 0b11,
}

/// Uses CMD6 to modify a field of the EXT_CSD.
pub fn modify_ext_csd(access_mode: AccessMode, index: u8, value: u8) -> Cmd6 {
    Cmd6 {
        access: access_mode as u8,
        index,
        value,
        cmd_set: 0,
    }
}

/// CMD8 — SEND_EXT_CSD (MMC version)
/// Reads 512‑byte EXT_CSD register.
pub struct Cmd8<'a> {
    pub buf: &'a mut Aligned<A4, [u8; 512]>,
}
impl<'a> Command for Cmd8<'a> {
    const INDEX: u8 = 8;
    type Resp<'b>
        = R1
    where
        Self: 'b;
    fn arg(&self) -> u32 {
        0
    }
}
impl<'a> BlockCommand for Cmd8<'a> {
    fn block_size(&self) -> BlockSize {
        block_size(512)
    }
    fn block_count(&self) -> u32 {
        1
    }
}
impl<'a> BlockReadCommand for Cmd8<'a> {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        &mut *self.buf
    }
}

/// CMD8: Device sends its EXT_CSD register as a block of data.
pub fn send_ext_csd(ext_csd: &mut ExtCSD) -> Cmd8<'_> {
    Cmd8 {
        buf: &mut ext_csd.inner,
    }
}

/// CMD35 — ERASE_GROUP_START
pub struct Cmd35 {
    pub addr: u32,
}
impl Command for Cmd35 {
    const INDEX: u8 = 35;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.addr
    }
}
impl ControlCommand for Cmd35 {}

/// CMD36 — ERASE_GROUP_END
pub struct Cmd36 {
    pub addr: u32,
}
impl Command for Cmd36 {
    const INDEX: u8 = 36;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.addr
    }
}
impl ControlCommand for Cmd36 {}

/// CMD39 — FAST_IO (rarely used)
pub struct Cmd39 {
    pub addr: u8,
    pub data: u8,
}
impl Command for Cmd39 {
    const INDEX: u8 = 39;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        ((self.addr as u32) << 8) | (self.data as u32)
    }
}
impl ControlCommand for Cmd39 {}

/// CMD40 — GO_IRQ_STATE (rare)
pub struct Cmd40;
impl Command for Cmd40 {
    const INDEX: u8 = 40;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd40 {}

//
//
// /// CMD14: Host reads the reversed bus testing data pattern from a card
// pub fn bustest_read() -> Cmd<R1> {
//     cmd(14, 0)
// }
//
// /// CMD19: Host sends bus test pattern to a card
// pub fn bustest_write() -> Cmd<R1> {
//     cmd(19, 0)
// }
//
// /// CMD23: Defines the number of blocks (read/write) for a block read or write
// /// operation
// pub fn set_block_count(blockcount: u16) -> Cmd<R1> {
//     cmd(23, blockcount as u32)
// }
//
// /// CMD35: Sets the address of the first erase group within a range to be
// /// selected for erase
// ///
// /// Address is either byte address or sector address (set in OCR)
// pub fn erase_group_start(address: u32) -> Cmd<R1> {
//     cmd(35, address)
// }
//
// /// CMD36: Sets the address of the last erase group within a continuous range to
// /// be selected for erase
// ///
// /// Address is either byte address or sector address (set in OCR)
// pub fn erase_group_end(address: u32) -> Cmd<R1> {
//     cmd(36, address)
// }

/// Type marker for eMMC-specific extensions.
#[derive(Clone, Copy, Default, Debug)]
pub struct EMMC;

impl OCR<EMMC> {
    /// OCR \[7\]. False for High Voltage, true for Dual voltage
    pub fn is_dual_voltage_card(&self) -> bool {
        self.0 & 0x0000_0080 != 0
    }
    /// OCR \[30:29\]. Access mode. Defines the addressing mode used between host and card
    ///
    /// 0b00: byte mode
    /// 0b10: sector mode
    pub fn access_mode(&self) -> u8 {
        ((self.0 & 0x6000_0000) >> 29) as u8
    }
}
impl fmt::Debug for OCR<EMMC> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("OCR: Operation Conditions Register")
            .field(
                "Dual Voltage",
                &if self.is_dual_voltage_card() {
                    "yes"
                } else {
                    "no"
                },
            )
            .field(
                "Access mode",
                &match self.access_mode() {
                    0b00 => "byte",
                    0b10 => "sector",
                    _ => "unknown",
                },
            )
            .field("Busy", &self.is_busy())
            .finish()
    }
}

/// All possible values of the CBX field of the CID register on eMMC devices.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum DeviceType {
    RemovableDevice = 0b00,
    BGA = 0b01,
    POP = 0b10,
    Unknown = 0b11,
}

impl CID<EMMC> {
    /// CBX field, indicating device type.
    pub fn device_type(&self) -> DeviceType {
        match self.bytes[1] & 0x3 {
            0b00 => DeviceType::RemovableDevice,
            0b01 => DeviceType::BGA,
            0b10 => DeviceType::POP,
            _ => DeviceType::POP,
        }
    }

    /// OID field, indicating OEM/Application ID.
    ///
    /// The OID number is controlled, defined and allocated to an eMMC manufacturer by JEDEC.
    pub fn oem_application_id(&self) -> u8 {
        self.bytes[2]
    }

    /// PNM field, indicating product name.
    pub fn product_name(&self) -> &str {
        str::from_utf8(&self.bytes[3..9]).unwrap_or("<ERR>")
    }

    /// PRV field, indicating product revision.
    ///
    /// The return value is a (major, minor) version tuple.
    pub fn product_revision(&self) -> (u8, u8) {
        let major = (self.bytes[9] & 0xF0) >> 4;
        let minor = self.bytes[9] & 0x0F;
        (major, minor)
    }

    /// PSN field, indicating product serial number.
    pub fn serial(&self) -> u32 {
        (self.inner() >> 16) as u32
    }

    /// MDT field, indicating manufacturing date.
    ///
    /// The return value is a (month, year) tuple where the month code has 1 = January and the year
    /// is an offset from either 1997 or 2013 depending on the value of `EXT_CSD_REV`.
    pub fn manufacturing_date(&self) -> (u8, u8) {
        let month = (self.inner() >> 12) as u8 & 0xF;
        let year = (self.inner() >> 8) as u8 & 0xF;
        (month, year)
    }
}
impl fmt::Debug for CID<EMMC> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CID: Card Identification")
            .field("Manufacturer ID", &self.manufacturer_id())
            .field("Device Type", &self.device_type())
            .field("OEM ID", &self.oem_application_id())
            .field("Product Name", &self.product_name())
            .field("Product Revision", &self.product_revision())
            .field("Product Serial Number", &self.serial())
            .field("Manufacturing Date", &self.manufacturing_date())
            .finish()
    }
}

impl CSD<EMMC> {
    /// Erase size (in blocks)
    ///
    /// Minimum number of write blocks that must be erased in a single erase
    /// command
    pub fn erase_size_blocks(&self) -> u32 {
        let erase_grp_size = (self.0 >> 42) & 0x1F;
        let erase_grp_mult = (self.0 >> 37) & 0x1F;

        (erase_grp_size as u32 + 1) * (erase_grp_mult as u32 + 1)
    }
}
impl fmt::Debug for CSD<EMMC> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CSD: Card Specific Data")
            .field("Transfer Rate", &self.transfer_rate())
            .field("Read I (@min VDD)", &self.read_current_minimum_vdd())
            .field("Write I (@min VDD)", &self.write_current_minimum_vdd())
            .field("Read I (@max VDD)", &self.read_current_maximum_vdd())
            .field("Write I (@max VDD)", &self.write_current_maximum_vdd())
            .field("Erase Size (Blocks)", &self.erase_size_blocks())
            .finish()
    }
}

impl CardStatus<EMMC> {
    /// If set, the Device did not switch to the expected mode as requested by the SWITCH command
    pub fn switch_error(&self) -> bool {
        self.0 & 0x80 != 0
    }
    /// If set, one of the exception bits in field EXCEPTION_EVENTS_STATUS was set to indicate some
    /// exception has occurred. Host should check that field to discover the exception that has
    /// occurred to understand what further actions are needed in order to clear this bit.
    pub fn exception_event(&self) -> bool {
        self.0 & 0x40 != 0
    }
}
impl fmt::Debug for CardStatus<EMMC> {
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
            .field("Erase sequence cleared", &self.erase_reset())
            .field("Card state", &self.state())
            .field("Buffer empty", &self.ready_for_data())
            .field("Switch error", &self.switch_error())
            .field("Exception event", &self.exception_event())
            .field("Card expects app cmd", &self.app_cmd())
            .finish()
    }
}

/// Extended Card Specific Data
///
/// Ref JEDEC 84-A43 Section 8.4
#[derive(Clone, Copy)]
pub struct ExtCSD {
    pub inner: Aligned<A4, [u8; 512]>,
}

impl Default for ExtCSD {
    fn default() -> Self {
        Self::new()
    }
}

impl ExtCSD {
    /// Create a new `ExtCSD`
    #[inline]
    pub const fn new() -> Self {
        Self {
            inner: Aligned([0u8; 512]),
        }
    }

    /// Read the little-endian 32-bit word at byte offset `i * 4`.
    fn inner_word(&self, i: usize) -> u32 {
        u32::from_le_bytes(self.inner[i * 4..i * 4 + 4].try_into().unwrap())
    }

    /// Read the single byte at EXT_CSD offset `i`.
    fn byte(&self, i: usize) -> u8 {
        self.inner[i]
    }

    pub fn boot_info(&self) -> u8 {
        self.byte(228)
    }
    pub fn sleep_awake_timeout(&self) -> u8 {
        self.byte(217)
    }
    pub fn sleep_notification_time(&self) -> u8 {
        self.byte(216)
    }
    pub fn sector_count(&self) -> u32 {
        // bytes [215:212], little-endian
        self.inner_word(53)
    }
    pub fn driver_strength(&self) -> u8 {
        self.byte(197)
    }
    pub fn card_type(&self) -> u8 {
        self.byte(196)
    }
    pub fn csd_structure_version(&self) -> u8 {
        self.byte(194)
    }
    pub fn extended_csd_revision(&self) -> u8 {
        self.byte(192)
    }
    pub fn data_sector_size(&self) -> u8 {
        self.byte(61)
    }
    pub fn secure_removal_type(&self) -> u8 {
        self.byte(16)
    }
}

impl fmt::Debug for ExtCSD {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Extended CSD")
            .field("Boot Info", &self.boot_info())
            .field("Sleep/Awake Timeout", &self.sleep_awake_timeout())
            .field("Sleep Notification Time", &self.sleep_notification_time())
            .field("Sector Count", &self.sector_count())
            .field("Driver Strength", &self.driver_strength())
            .field("Card Type", &self.card_type())
            .field("CSD Structure Version", &self.csd_structure_version())
            .field("Extended CSD Revision", &self.extended_csd_revision())
            .field("Sector Size", &self.data_sector_size())
            .field("Secure removal type", &self.secure_removal_type())
            .finish()
    }
}

/// eMMC hosts need to be able to create relative card addresses so that they can be assigned to
/// devices. SD hosts only ever retrieve RCAs from 32-bit card responses.
impl From<u16> for RCA<EMMC> {
    fn from(address: u16) -> Self {
        Self((address as u32) << 16, PhantomData)
    }
}

#[derive(Clone, Copy, Debug, Default)]
/// eMMC storage
pub struct Emmc {
    /// Operation Conditions Register
    pub ocr: OCR<EMMC>,
    /// Card ID
    pub cid: CID<EMMC>,
    /// Card Specific Data
    pub csd: CSD<EMMC>,
    /// Extended Card Specific Data
    pub ext_csd: ExtCSD,
}

impl Addressable for Emmc {
    type Ext = EMMC;

    /// Is this a standard or high capacity peripheral?
    fn get_capacity(&self) -> CardCapacity {
        if self.ocr.access_mode() == 0b10 {
            CardCapacity::HighCapacity
        } else {
            CardCapacity::StandardCapacity
        }
    }

    fn block_count(&self) -> u32 {
        self.ext_csd.sector_count()
    }

    fn supports_cmd23(&self) -> bool {
        true // mandatory on eMMC since spec v4.1
    }

    fn supports_acmd23(&self) -> bool {
        false // app commands are not supported
    }
}

impl Acquirable for Emmc {
    async fn acquire<B: MmcBus, D: DelayNs>(
        bus: &mut BusAdapter<B, D>,
        block_size: BlockSize,
        freq: u32,
    ) -> Result<Self, MmcError> {
        let mut this = Self::default();

        if block_size.len() != 512 {
            // eMMC requires 512 block size
            return Err(MmcError::Other);
        }

        // Get the bus width configured in the Sdmmc peripheral
        let bus_width = bus.bus.supports_bus_width();

        let high_voltage = 0b0 << 7;
        let access_mode = 0b10 << 29;
        let op_cond = high_voltage | access_mode | 0b1_1111_1111 << 15;

        this.ocr = bus.get_ocr(&send_op_cond(op_cond), false).await?;

        this.cid = bus
            .send_command(common::all_send_cid(), false)
            .await?
            .into();

        bus.rca = 1u16;

        bus.send_command(assign_relative_address(bus.rca), false)
            .await?;

        this.csd = bus
            .send_command(common::send_csd(bus.rca), false)
            .await?
            .into();

        bus.select_card(Some(bus.rca)).await?;

        let widbus = match bus_width {
            BusWidth::W1 => 0,
            BusWidth::W4 => 1,
            BusWidth::W8 => 2,
        };

        bus.send_command(modify_ext_csd(AccessMode::WriteByte, 183, widbus), false)
            .await?;

        bus.bus.set_bus(bus_width, freq)?;

        bus.read_blocks(send_ext_csd(&mut this.ext_csd), false)
            .await?;

        Ok(this)
    }
}

/// Card Storage Device
impl<B: MmcBus, D: DelayNs, const BLOCK_SIZE: usize> BlockDevice<Emmc, B, D, BLOCK_SIZE> {
    /// Create a new SD card
    pub async fn new_emmc(bus: B, freq: u32, delay: D) -> Result<Self, MmcError> {
        Self::new(bus, delay, freq).await
    }

    /// Create a uninit SD card
    pub fn new_uninit_emmc(bus: B, delay: D) -> Self {
        Self::new_uninit(bus, delay)
    }
}
