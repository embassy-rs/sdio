use core::marker::PhantomData;
use core::{fmt, mem, slice};

use core::fmt::Debug;

use aligned::{A4, Aligned};

use crate::{
    BlockCommand, BlockReadCommand, BlockWriteCommand, Command, ControlCommand, R0, R1, R1b, R2,
    R3, R4, R6,
};

// ============================================================================
// COMMON COMMANDS
// ============================================================================

/// CMD0 — GO_IDLE_STATE
pub struct Cmd0;

impl Command for Cmd0 {
    const INDEX: u8 = 0;
    type Resp<'a> = R0;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd0 {}

/// CMD0 — GO_IDLE_STATE
pub fn idle() -> Cmd0 {
    Cmd0
}

/// CMD0 — GO_IDLE_STATE (SPI)
pub struct Cmd0S;

impl Command for Cmd0S {
    const INDEX: u8 = 0;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd0S {}

/// CMD0 — GO_IDLE_STATE
pub fn idle_spi() -> Cmd0S {
    Cmd0S
}

/// CMD2 — ALL_SEND_CID
pub struct Cmd2;
impl Command for Cmd2 {
    const INDEX: u8 = 2;
    type Resp<'a> = R2;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd2 {}

/// CMD2: Ask any card to send their CID
pub fn all_send_cid() -> Cmd2 {
    Cmd2
}

/// CMD7 — SELECT/DESELECT_CARD
pub struct Cmd7 {
    pub rca: u16,
}
impl Command for Cmd7 {
    const INDEX: u8 = 7;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        (self.rca as u32) << 16
    }
}
impl ControlCommand for Cmd7 {}

/// CMD7: Select or deselect card
pub fn select_card(rca: u16) -> Cmd7 {
    Cmd7 { rca }
}

/// CMD9 — SEND_CSD
pub struct Cmd9 {
    pub rca: u16,
}
impl Command for Cmd9 {
    const INDEX: u8 = 9;
    type Resp<'a> = R2;
    fn arg(&self) -> u32 {
        (self.rca as u32) << 16
    }
}
impl ControlCommand for Cmd9 {}

/// CMD9: Send CSD
pub fn send_csd(rca: u16) -> Cmd9 {
    Cmd9 { rca }
}

/// CMD10 — SEND_CID
pub struct Cmd10 {
    pub rca: u16,
}
impl Command for Cmd10 {
    const INDEX: u8 = 10;
    type Resp<'a> = R2;
    fn arg(&self) -> u32 {
        (self.rca as u32) << 16
    }
}
impl ControlCommand for Cmd10 {}

/// CMD10: Send CID
pub fn send_cid(rca: u16) -> Cmd10 {
    Cmd10 { rca }
}

/// CMD12 — STOP_TRANSMISSION (R1b)
pub struct Cmd12;
impl Command for Cmd12 {
    const INDEX: u8 = 12;
    type Resp<'a> = R1b;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd12 {}

/// CMD12: Stop transmission
pub fn stop_transmission() -> Cmd12 {
    Cmd12
}

/// CMD13 — SEND_STATUS
pub struct Cmd13 {
    pub rca: u16,
    pub task_status: bool,
}
impl Command for Cmd13 {
    const INDEX: u8 = 13;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        (self.rca as u32) << 16 | (self.task_status as u32) << 15
    }
}
impl ControlCommand for Cmd13 {}

/// CMD13: Ask card to send status or task status
pub fn card_status(rca: u16, task_status: bool) -> Cmd13 {
    Cmd13 { rca, task_status }
}

// /// CMD15: Sends card to inactive state
// pub fn go_inactive_state(rca: u16) -> Cmd<Rz> {
//     cmd(15, u32::from(rca) << 16)
// }
//

/// CMD16 — SET_BLOCKLEN (rarely used on SDHC/SDXC)
pub struct Cmd16 {
    pub block_len: u32,
}
impl Command for Cmd16 {
    const INDEX: u8 = 16;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        self.block_len
    }
}
impl ControlCommand for Cmd16 {}

/// CMD16: Set block len
pub fn set_block_length(block_len: u32) -> Cmd16 {
    Cmd16 { block_len }
}

/// CMD17 — READ_SINGLE_BLOCK
pub struct Cmd17<'a, const BLOCK_SIZE: usize> {
    pub addr: u32,
    pub buf: &'a mut Aligned<A4, [u8; BLOCK_SIZE]>,
}
impl<'a, const BLOCK_SIZE: usize> Command for Cmd17<'a, BLOCK_SIZE> {
    const INDEX: u8 = 17;
    type Resp<'b>
        = R1
    where
        Self: 'b;
    fn arg(&self) -> u32 {
        self.addr
    }
}
impl<'a, const BLOCK_SIZE: usize> BlockCommand for Cmd17<'a, BLOCK_SIZE> {
    fn block_size(&self) -> BlockSize {
        block_size(BLOCK_SIZE)
    }
    fn block_count(&self) -> u32 {
        1
    }
}
impl<'a, const BLOCK_SIZE: usize> BlockReadCommand for Cmd17<'a, BLOCK_SIZE> {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        &mut *self.buf
    }
}

/// CMD17: Read a single block from the card
pub fn read_single_block<const BLOCK_SIZE: usize>(
    addr: u32,
    buf: &mut Aligned<A4, [u8; BLOCK_SIZE]>,
) -> Cmd17<'_, BLOCK_SIZE> {
    Cmd17 { addr, buf }
}

/// CMD18 — READ_MULTIPLE_BLOCK
pub struct Cmd18<'a, const BLOCK_SIZE: usize> {
    pub addr: u32,
    pub buf: &'a mut [Aligned<A4, [u8; BLOCK_SIZE]>],
}
impl<'a, const BLOCK_SIZE: usize> Command for Cmd18<'a, BLOCK_SIZE> {
    const INDEX: u8 = 18;
    type Resp<'b>
        = R1
    where
        Self: 'b;
    fn arg(&self) -> u32 {
        self.addr
    }
}
impl<'a, const BLOCK_SIZE: usize> BlockCommand for Cmd18<'a, BLOCK_SIZE> {
    fn block_size(&self) -> BlockSize {
        block_size(BLOCK_SIZE)
    }

    fn block_count(&self) -> u32 {
        self.buf.len() as u32
    }
}
impl<'a, const BLOCK_SIZE: usize> BlockReadCommand for Cmd18<'a, BLOCK_SIZE> {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        unsafe {
            mem::transmute(slice::from_raw_parts_mut(
                self.buf.as_mut_ptr() as *mut _,
                size_of_val(self.buf),
            ))
        }
    }
}

/// CMD18: Read multiple block from the card
pub fn read_multiple_blocks<const BLOCK_SIZE: usize>(
    addr: u32,
    buf: &mut [Aligned<A4, [u8; BLOCK_SIZE]>],
) -> Cmd18<'_, BLOCK_SIZE> {
    Cmd18 { addr, buf }
}

/// CMD24 — WRITE_BLOCK
pub struct Cmd24<'a, const BLOCK_SIZE: usize> {
    pub addr: u32,
    pub buf: &'a Aligned<A4, [u8; BLOCK_SIZE]>,
}
impl<'a, const BLOCK_SIZE: usize> Command for Cmd24<'a, BLOCK_SIZE> {
    const INDEX: u8 = 24;
    type Resp<'b>
        = R1b
    where
        Self: 'b;
    fn arg(&self) -> u32 {
        self.addr
    }
}
impl<'a, const BLOCK_SIZE: usize> BlockCommand for Cmd24<'a, BLOCK_SIZE> {
    fn block_size(&self) -> BlockSize {
        block_size(BLOCK_SIZE)
    }
    fn block_count(&self) -> u32 {
        1
    }
}
impl<'a, const BLOCK_SIZE: usize> BlockWriteCommand for Cmd24<'a, BLOCK_SIZE> {
    fn buf(&self) -> &Aligned<A4, [u8]> {
        self.buf
    }
}

/// CMD24: Write block
pub fn write_single_block<const BLOCK_SIZE: usize>(
    addr: u32,
    buf: &Aligned<A4, [u8; BLOCK_SIZE]>,
) -> Cmd24<'_, BLOCK_SIZE> {
    Cmd24 { addr, buf }
}

/// CMD25 — WRITE_MULTIPLE_BLOCK
pub struct Cmd25<'a, const BLOCK_SIZE: usize> {
    pub addr: u32,
    pub buf: &'a [Aligned<A4, [u8; BLOCK_SIZE]>],
}
impl<'a, const BLOCK_SIZE: usize> Command for Cmd25<'a, BLOCK_SIZE> {
    const INDEX: u8 = 25;
    type Resp<'b>
        = R1b
    where
        Self: 'b;
    fn arg(&self) -> u32 {
        self.addr
    }
}
impl<'a, const BLOCK_SIZE: usize> BlockCommand for Cmd25<'a, BLOCK_SIZE> {
    fn block_size(&self) -> BlockSize {
        block_size(BLOCK_SIZE)
    }
    fn block_count(&self) -> u32 {
        self.buf.len() as u32
    }
}
impl<'a, const BLOCK_SIZE: usize> BlockWriteCommand for Cmd25<'a, BLOCK_SIZE> {
    fn buf(&self) -> &Aligned<A4, [u8]> {
        unsafe {
            mem::transmute(slice::from_raw_parts(
                self.buf.as_ptr() as *const _,
                size_of_val(self.buf),
            ))
        }
    }
}

/// CMD25: Write multiple blocks
pub fn write_multiple_blocks<const BLOCK_SIZE: usize>(
    addr: u32,
    buf: &[Aligned<A4, [u8; BLOCK_SIZE]>],
) -> Cmd25<'_, BLOCK_SIZE> {
    Cmd25 { addr, buf }
}

// /// CMD27: Program CSD
// pub fn program_csd() -> Cmd<R1> {
//     cmd(27, 0)
// }

/// CMD38 — ERASE (R1b)
pub struct Cmd38;
impl Command for Cmd38 {
    const INDEX: u8 = 38;
    type Resp<'a> = R1b;
    fn arg(&self) -> u32 {
        0
    }
}
impl ControlCommand for Cmd38 {}

/// CMD38: Erase all previously selected write blocks
pub fn erase() -> Cmd38 {
    Cmd38
}

/// CMD55 — APP_CMD prefix
pub struct Cmd55 {
    pub rca: u16,
}
impl Command for Cmd55 {
    const INDEX: u8 = 55;
    type Resp<'a> = R1;
    fn arg(&self) -> u32 {
        (self.rca as u32) << 16
    }
}
impl ControlCommand for Cmd55 {}

/// CMD55: App Command. Indicates that next command will be a app command
pub fn app_cmd(rca: u16) -> Cmd55 {
    Cmd55 { rca }
}

/// Types of SD Card
#[derive(Debug, Copy, Clone)]
#[non_exhaustive]
#[derive(Default)]
pub enum CardCapacity {
    /// SDSC / Standard Capacity (<= 2GB)
    #[default]
    StandardCapacity,
    /// SDHC / High capacity (<= 32GB for SD cards, <= 256GB for eMMC)
    HighCapacity,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum BlockSize {
    B1,
    B2,
    B4,
    B8,
    B16,
    B32,
    B64,
    B128,
    B256,
    B512,
    B1024,
    B2048,
    B4096,
    B8192,
    B16kB,
    Unknown,
}

impl BlockSize {
    /// Length of the block size. Will return 0 if unknown.
    #[allow(clippy::len_without_is_empty)]
    pub const fn len(&self) -> usize {
        match self {
            BlockSize::B1 => 1,
            BlockSize::B2 => 2,
            BlockSize::B4 => 4,
            BlockSize::B8 => 8,
            BlockSize::B16 => 16,
            BlockSize::B32 => 32,
            BlockSize::B64 => 64,
            BlockSize::B128 => 128,
            BlockSize::B256 => 256,
            BlockSize::B512 => 512,
            BlockSize::B1024 => 1024,
            BlockSize::B2048 => 2048,
            BlockSize::B4096 => 4096,
            BlockSize::B8192 => 8192,
            BlockSize::B16kB => 16384,
            _ => 0,
        }
    }
}

pub(crate) const fn block_size(len: usize) -> BlockSize {
    match len {
        1 => BlockSize::B1,
        2 => BlockSize::B2,
        4 => BlockSize::B4,
        8 => BlockSize::B8,
        16 => BlockSize::B16,
        32 => BlockSize::B32,
        64 => BlockSize::B64,
        128 => BlockSize::B128,
        256 => BlockSize::B256,
        512 => BlockSize::B512,
        1024 => BlockSize::B1024,
        2048 => BlockSize::B2048,
        4096 => BlockSize::B4096,
        8192 => BlockSize::B8192,
        16384 => BlockSize::B16kB,
        _ => BlockSize::Unknown,
    }
}

/// CURRENT_STATE enum. Used for R1 response in command queue mode in SD spec, or all R1 responses
/// in eMMC spec.
///
/// Ref PLSS_v7_10 Table 4-75
/// Ref JESD84-B51 Table 68
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(dead_code)]
pub enum CurrentState {
    /// Card state is ready
    Ready = 1,
    /// Card is in identification state
    Identification = 2,
    /// Card is in standby state
    Standby = 3,
    /// Card is in transfer state
    Transfer = 4,
    /// Card is sending an operation
    Sending = 5,
    /// Card is receiving operation information
    Receiving = 6,
    /// Card is in programming state
    Programming = 7,
    /// Card is disconnected
    Disconnected = 8,
    /// Card is in bus testing mode. Only valid for eMMC (reserved by SD spec).
    BusTest = 9,
    /// Card is in sleep mode. Only valid for eMMC (reserved by SD spec).
    Sleep = 10,
    // 11 - 15: Reserved
    /// Error
    Error = 128,
}

impl From<u8> for CurrentState {
    fn from(n: u8) -> Self {
        match n {
            1 => Self::Ready,
            2 => Self::Identification,
            3 => Self::Standby,
            4 => Self::Transfer,
            5 => Self::Sending,
            6 => Self::Receiving,
            7 => Self::Programming,
            8 => Self::Disconnected,
            9 => Self::BusTest,
            10 => Self::Sleep,
            _ => Self::Error,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
#[allow(non_camel_case_types)]
pub enum CurrentConsumption {
    I_0mA,
    I_1mA,
    I_5mA,
    I_10mA,
    I_25mA,
    I_35mA,
    I_45mA,
    I_60mA,
    I_80mA,
    I_100mA,
    I_200mA,
}
impl From<&CurrentConsumption> for u32 {
    fn from(i: &CurrentConsumption) -> u32 {
        match i {
            CurrentConsumption::I_0mA => 0,
            CurrentConsumption::I_1mA => 1,
            CurrentConsumption::I_5mA => 5,
            CurrentConsumption::I_10mA => 10,
            CurrentConsumption::I_25mA => 25,
            CurrentConsumption::I_35mA => 35,
            CurrentConsumption::I_45mA => 45,
            CurrentConsumption::I_60mA => 60,
            CurrentConsumption::I_80mA => 80,
            CurrentConsumption::I_100mA => 100,
            CurrentConsumption::I_200mA => 200,
        }
    }
}
impl CurrentConsumption {
    fn from_minimum_reg(reg: u128) -> CurrentConsumption {
        match reg & 0x7 {
            0 => CurrentConsumption::I_0mA,
            1 => CurrentConsumption::I_1mA,
            2 => CurrentConsumption::I_5mA,
            3 => CurrentConsumption::I_10mA,
            4 => CurrentConsumption::I_25mA,
            5 => CurrentConsumption::I_35mA,
            6 => CurrentConsumption::I_60mA,
            _ => CurrentConsumption::I_100mA,
        }
    }
    fn from_maximum_reg(reg: u128) -> CurrentConsumption {
        match reg & 0x7 {
            0 => CurrentConsumption::I_1mA,
            1 => CurrentConsumption::I_5mA,
            2 => CurrentConsumption::I_10mA,
            3 => CurrentConsumption::I_25mA,
            4 => CurrentConsumption::I_35mA,
            5 => CurrentConsumption::I_45mA,
            6 => CurrentConsumption::I_80mA,
            _ => CurrentConsumption::I_200mA,
        }
    }
}
impl fmt::Debug for CurrentConsumption {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ma: u32 = self.into();
        write!(f, "{} mA", ma)
    }
}

/// Operation Conditions Register (OCR)
///
/// R3
#[derive(Clone, Copy, Default)]
pub struct OCR<Ext>(pub(crate) u32, PhantomData<Ext>);
impl<Ext> From<R3> for OCR<Ext> {
    fn from(resp: R3) -> Self {
        Self(resp.ocr, PhantomData)
    }
}
impl<Ext> From<R4> for OCR<Ext> {
    fn from(resp: R4) -> Self {
        Self(resp.ocr, PhantomData)
    }
}
impl<Ext> OCR<Ext> {
    /// Card power up status bit (busy)
    pub fn is_busy(&self) -> bool {
        self.0 & 0x8000_0000 == 0 // Set active LOW
    }
}

/// Card Identification Register (CID)
///
/// R2
#[derive(Clone, Copy, Default)]
pub struct CID<Ext> {
    pub(crate) bytes: [u8; 16],
    ext: PhantomData<Ext>,
}
/// From little endian words
impl<Ext> From<R2> for CID<Ext> {
    fn from(resp: R2) -> Self {
        let words = resp.words;
        let inner = ((words[3] as u128) << 96)
            | ((words[2] as u128) << 64)
            | ((words[1] as u128) << 32)
            | words[0] as u128;

        Self {
            bytes: inner.to_be_bytes(),
            ext: PhantomData,
        }
    }
}
impl<Ext> CID<Ext> {
    pub(crate) const fn inner(&self) -> u128 {
        u128::from_be_bytes(self.bytes)
    }

    /// Manufacturer ID
    pub fn manufacturer_id(&self) -> u8 {
        self.bytes[0]
    }
    #[allow(unused)]
    fn crc7(&self) -> u8 {
        (self.bytes[15] >> 1) & 0x7F
    }
}

/// Card Specific Data (CSD)
#[derive(Clone, Copy, Default)]
pub struct CSD<Ext>(pub(crate) u128, PhantomData<Ext>);
impl<Ext> From<u128> for CSD<Ext> {
    fn from(inner: u128) -> Self {
        Self(inner, PhantomData)
    }
}
/// From little endian words
impl<Ext> From<R2> for CSD<Ext> {
    fn from(resp: R2) -> Self {
        let words = resp.words;

        let inner = ((words[3] as u128) << 96)
            | ((words[2] as u128) << 64)
            | ((words[1] as u128) << 32)
            | words[0] as u128;
        inner.into()
    }
}

impl<Ext> CSD<Ext> {
    /// CSD structure version
    pub fn version(&self) -> u8 {
        (self.0 >> 126) as u8 & 3
    }
    /// Maximum data transfer rate per one data line
    pub fn transfer_rate(&self) -> u8 {
        (self.0 >> 96) as u8
    }
    /// Maximum block length. In an SD Memory Card the WRITE_BL_LEN is
    /// always equal to READ_BL_LEN
    pub fn block_length(&self) -> BlockSize {
        // Read block length
        match (self.0 >> 80) & 0xF {
            0 => BlockSize::B1,
            1 => BlockSize::B2,
            2 => BlockSize::B4,
            3 => BlockSize::B8,
            4 => BlockSize::B16,
            5 => BlockSize::B32,
            6 => BlockSize::B64,
            7 => BlockSize::B128,
            8 => BlockSize::B256,
            9 => BlockSize::B512,
            10 => BlockSize::B1024,
            11 => BlockSize::B2048,
            12 => BlockSize::B4096,
            13 => BlockSize::B8192,
            14 => BlockSize::B16kB,
            _ => BlockSize::Unknown,
        }
    }
    /// Maximum read current at the minimum VDD
    pub fn read_current_minimum_vdd(&self) -> CurrentConsumption {
        CurrentConsumption::from_minimum_reg((self.0 >> 59) & 0x7)
    }
    /// Maximum write current at the minimum VDD
    pub fn write_current_minimum_vdd(&self) -> CurrentConsumption {
        CurrentConsumption::from_minimum_reg((self.0 >> 56) & 0x7)
    }
    /// Maximum read current at the maximum VDD
    pub fn read_current_maximum_vdd(&self) -> CurrentConsumption {
        CurrentConsumption::from_maximum_reg((self.0 >> 53) & 0x7)
    }
    /// Maximum write current at the maximum VDD
    pub fn write_current_maximum_vdd(&self) -> CurrentConsumption {
        CurrentConsumption::from_maximum_reg((self.0 >> 50) & 0x7)
    }
}

/// Card Status (R1)
///
/// Error and state information of an executed command
///
/// Ref PLSS_v7_10 Section 4.10.1
#[derive(Clone, Copy)]
pub struct CardStatus<Ext>(pub(crate) u32, PhantomData<Ext>);

impl<Ext> From<R1> for CardStatus<Ext> {
    fn from(resp: R1) -> Self {
        Self(resp.status, PhantomData)
    }
}

impl<Ext> CardStatus<Ext> {
    /// Command's argument was out of range
    pub fn out_of_range(&self) -> bool {
        self.0 & 0x8000_0000 != 0
    }
    /// Misaligned address
    pub fn address_error(&self) -> bool {
        self.0 & 0x4000_0000 != 0
    }
    /// Block len error
    pub fn block_len_error(&self) -> bool {
        self.0 & 0x2000_0000 != 0
    }
    /// Error in the erase commands sequence
    pub fn erase_seq_error(&self) -> bool {
        self.0 & 0x1000_0000 != 0
    }
    /// Invalid selection of blocks for erase
    pub fn erase_param(&self) -> bool {
        self.0 & 0x800_0000 != 0
    }
    /// Host attempted to write to protected area
    pub fn wp_violation(&self) -> bool {
        self.0 & 0x400_0000 != 0
    }
    /// Card is locked by the host
    pub fn card_is_locked(&self) -> bool {
        self.0 & 0x200_0000 != 0
    }
    /// Password error
    pub fn lock_unlock_failed(&self) -> bool {
        self.0 & 0x100_0000 != 0
    }
    /// Crc check of previous command failed
    pub fn com_crc_error(&self) -> bool {
        self.0 & 0x80_0000 != 0
    }
    /// Command is not legal for the card state
    pub fn illegal_command(&self) -> bool {
        self.0 & 0x40_0000 != 0
    }
    /// Card internal ECC failed
    pub fn card_ecc_failed(&self) -> bool {
        self.0 & 0x20_0000 != 0
    }
    /// Internal controller error
    pub fn cc_error(&self) -> bool {
        self.0 & 0x10_0000 != 0
    }
    /// A General error occurred
    pub fn error(&self) -> bool {
        self.0 & 0x8_0000 != 0
    }
    /// CSD error
    pub fn csd_overwrite(&self) -> bool {
        self.0 & 0x1_0000 != 0
    }
    /// Some blocks where skipped while erasing
    pub fn wp_erase_skip(&self) -> bool {
        self.0 & 0x8000 != 0
    }
    /// Erase sequence was aborted
    pub fn erase_reset(&self) -> bool {
        self.0 & 0x2000 != 0
    }
    /// Current card state
    pub fn state(&self) -> CurrentState {
        CurrentState::from(((self.0 >> 9) & 0xF) as u8)
    }
    /// Corresponds to buffer empty signaling on the bus
    pub fn ready_for_data(&self) -> bool {
        self.0 & 0x100 != 0
    }
    /// The card will accept a ACMD
    pub fn app_cmd(&self) -> bool {
        self.0 & 0x20 != 0
    }
}

/// Relative Card Address (RCA)
///
/// R6
#[derive(Debug, Copy, Clone, Default)]
pub struct RCA<Ext>(pub(crate) u32, pub(crate) PhantomData<Ext>);
impl<Ext> From<R6> for RCA<Ext> {
    fn from(resp: R6) -> Self {
        Self(
            ((resp.rca as u32) << 16) | (resp.status as u32),
            PhantomData,
        )
    }
}
impl<Ext> RCA<Ext> {
    /// Address of card
    pub fn address(&self) -> u16 {
        (self.0 >> 16) as u16
    }
}
