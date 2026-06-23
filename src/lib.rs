//! SD Card Registers
//!
//! Register representations can be created from an array of little endian
//! words. Note that the SDMMC protocol transfers the registers in big endian
//! byte order.
//!
//! PLSS_v7_10: Physical Layer Specification Simplified Specification Version
//! 7.10. March 25, 2020. (C) SD Card Association

#![cfg_attr(not(test), no_std)]

use aligned::{A4, Aligned};
use embedded_hal_async::delay::DelayNs;

use crate::sd::{
    BlockSize, CardCapacity, CardStatus, OCR, block_size, read_multiple_blocks, read_single_block,
    stop_transmission, write_multiple_blocks, write_single_block,
};

pub mod common;
pub mod emmc;
pub mod sd;
pub mod sdio;
pub mod spi;

const INIT_FREQ: u32 = 400_000;

#[derive(Debug)]
pub enum MmcError {
    Timeout,
    Crc,
    IllegalCommand,
    Busy,
    Io,
    SignalingSwitchFailed,
    Unsupported,
    Other,
}

#[derive(Debug, Clone, Copy)]
pub enum BusWidth {
    W1,
    W4,
    W8,
}

#[derive(Debug, Clone, Copy)]
pub enum ResponseLen {
    Zero,
    R48,
    R136,
}

impl ResponseLen {
    pub const fn words(&self) -> usize {
        match self {
            Self::Zero => 0,
            Self::R48 => 1,
            Self::R136 => 4,
        }
    }
}

/// ---------------------------------------------------------------------------
/// Response Trait
/// ---------------------------------------------------------------------------
///
/// Represents a parsed response from the card.
/// Each command defines its own associated response type via a GAT.
/// This allows strongly typed responses (R1, R2, R3, R5, R6, R7, etc.)
/// instead of a generic "MmcResponse" blob.
///
/// Example:
///   CMD8 → R7
///   CMD17 → R1
///   CMD9  → R2
///
pub trait Response: Sized {
    /// Whether to check the CRC
    const CRC: bool;

    /// Whether to wait until DAT0 goes high
    const BUSY: bool;

    /// Response length
    const LEN: ResponseLen;

    /// Parse the reponse from words. Only the `ResponseLen` words are used.
    fn from_words(buf: &[u32; 4]) -> Self;
}

/// ---------------------------------------------------------------------------
/// Command Trait (with GAT for response type)
/// ---------------------------------------------------------------------------
///
/// Represents a protocol command (CMD0–CMD63, ACMDs, CMD52/53 for SDIO).
///
/// - `INDEX` is the fixed command index from the SD/MMC/SDIO spec.
/// - `arg()` returns the 32-bit argument field.
/// - `Resp<'a>` is the associated response type for this command.
///
/// This gives you compile-time correctness:
///   - CMD8 always returns R7
///   - CMD17 always returns R1
///   - CMD9 always returns R2
///
/// No downcasting, no runtime parsing, no mistakes.
///
pub trait Command {
    /// The fixed command index (e.g., 17 for CMD17).
    const INDEX: u8;

    /// The associated response type for this command.
    type Resp<'a>: Response
    where
        Self: 'a;

    /// The fixed command index (e.g., 17 for CMD17).
    fn index(&self) -> u8 {
        Self::INDEX
    }

    /// Compute the 32-bit argument for this command.
    fn arg(&self) -> u32;
}

/// Block mode: fixed-size blocks (CMD17/18/24/25, CMD53 block mode)
pub trait BlockCommand: Command {
    /// Size of each block in bytes (usually 512 for SD/MMC).
    fn block_size(&self) -> BlockSize;

    /// Number of blocks to transfer.
    fn block_count(&self) -> u32;
}

/// Byte mode: arbitrary byte counts (CMD53 byte mode, SPI multi-byte)
pub trait ByteCommand: Command {
    /// Number of bytes to transfer (arbitrary length).
    fn byte_count(&self) -> usize;
}

/// ControlCommand: commands with no data transfer (CMD0, CMD8, CMD55, etc.)
pub trait ControlCommand: Command {}

/// BlockReadCommand: block-mode read (CMD17, CMD18)
pub trait BlockReadCommand: BlockCommand {
    /// Mutable buffer for block-mode reads. The length of this buffer must be `block_size()` * `block_count()`
    fn buf(&mut self) -> &mut Aligned<A4, [u8]>;
}

/// BlockWriteCommand: block-mode write (CMD24, CMD25)
pub trait BlockWriteCommand: BlockCommand {
    /// Buffer for block-mode writes. The length of this buffer must be `block_size()` * `block_count()`
    fn buf(&self) -> &Aligned<A4, [u8]>;
}

/// ByteReadCommand: byte-mode read (CMD53 byte read)
pub trait ByteReadCommand: ByteCommand {
    /// Mutable buffer for byte-mode reads. The length of this buffer must be `byte_count()`.
    fn buf(&mut self) -> &mut Aligned<A4, [u8]>;
}

/// ByteWriteCommand: byte-mode write (CMD53 byte write)
pub trait ByteWriteCommand: ByteCommand {
    /// Buffer for byte-mode writes. The length of this buffer must be `byte_count()`.
    fn buf(&self) -> &Aligned<A4, [u8]>;
}

/// ---------------------------------------------------------------------------
/// MmcBus Trait
/// ---------------------------------------------------------------------------
///
/// This is the lowest-level hardware abstraction for SD/MMC/SDIO host
/// controllers. It corresponds to the Linux `mmc_host_ops` interface.
///
/// It exposes:
///   • Command-only operations
///   • Block-mode read/write
///   • Byte-mode read/write
///   • Bus configuration (clock, width)
///
/// Everything else (card initialization, SDIO function drivers, block devices)
/// is built on top of this trait.
///
/// If hardware support is available, methods should not return until DAT0 goes high
/// if the associated reponse has `BUSY` set to `true`.
///
pub trait MmcBus {
    /// Send a command that has no data transfer (e.g., CMD0, CMD8, CMD55).
    ///
    /// If called with `CMD11`, the bus should perform the voltage switch sequence.
    fn send_command<'a, C>(
        &mut self,
        cmd: C,
    ) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: ControlCommand + 'a;

    /// Read N blocks of fixed size (CMD17, CMD18, CMD53 block mode).
    fn read_blocks<'a, C>(&mut self, cmd: C) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: BlockReadCommand + 'a;

    /// Write N blocks of fixed size (CMD24, CMD25, CMD53 block mode).
    fn write_blocks<'a, C>(
        &mut self,
        cmd: C,
    ) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: BlockWriteCommand + 'a;

    /// Read an arbitrary number of bytes (CMD53 byte mode, SPI multi-byte).
    fn read_bytes<'a, C>(&mut self, cmd: C) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: ByteReadCommand + 'a;

    /// Write an arbitrary number of bytes (CMD53 byte mode, SPI multi-byte).
    fn write_bytes<'a, C>(&mut self, cmd: C) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: ByteWriteCommand + 'a;

    /// Initialize the bus in one-bit mode at 3.3v and the requested frequency.
    ///
    /// `hz` will always be `400_000`. The argument is provided so that the HAL does not have to define this.
    fn init_idle(&mut self, hz: u32) -> impl Future<Output = Result<(), MmcError>>;

    /// Tune the bus, if required. Called after the bus is set to the target frequency; needed for uhs.
    #[allow(unused_variables)]
    fn tune_bus<O>(
        &mut self,
        width: BusWidth,
        hz: u32,
        op: &mut O,
    ) -> impl Future<Output = Result<(), MmcError>>
    where
        O: TuningOp,
    {
        async { Ok(()) }
    }

    /// Configure bus width and frequency.
    ///
    /// Will not be called with a frequency higher than `supports_frequency()` or a bus width above
    /// `supports_bus_width()`.
    fn set_bus(&mut self, width: BusWidth, hz: u32) -> Result<(), MmcError>;

    /// Optional: whether the host supports native MMC mode. Otherwise, SPI mode is used.
    fn supports_mmc(&self) -> bool {
        false
    }

    /// Optional: the maximum bus width available to the host
    fn supports_bus_width(&self) -> BusWidth {
        BusWidth::W1
    }

    /// Optional: whether the host supports 1.8v. If true, `send_command` will be called with `CMD11`.
    fn supports_1v8(&self) -> bool {
        false
    }

    /// Optional: the maximum frequency supported by this bus. Defaults to 25Mhz
    fn supports_frequency(&self) -> u32 {
        25_000_000
    }
}

impl<T: MmcBus> MmcBus for &mut T {
    async fn send_command<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ControlCommand + 'a,
    {
        T::send_command(self, cmd).await
    }

    async fn read_blocks<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockReadCommand + 'a,
    {
        T::read_blocks(self, cmd).await
    }

    async fn write_blocks<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockWriteCommand + 'a,
    {
        T::write_blocks(self, cmd).await
    }

    async fn read_bytes<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ByteReadCommand + 'a,
    {
        T::read_bytes(self, cmd).await
    }

    async fn write_bytes<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ByteWriteCommand + 'a,
    {
        T::write_bytes(self, cmd).await
    }

    async fn tune_bus<O>(&mut self, width: BusWidth, hz: u32, op: &mut O) -> Result<(), MmcError>
    where
        O: TuningOp,
    {
        T::tune_bus(self, width, hz, op).await
    }

    async fn init_idle(&mut self, hz: u32) -> Result<(), MmcError> {
        T::init_idle(self, hz).await
    }

    fn set_bus(&mut self, width: BusWidth, hz: u32) -> Result<(), MmcError> {
        T::set_bus(self, width, hz)
    }

    fn supports_1v8(&self) -> bool {
        T::supports_1v8(self)
    }

    fn supports_bus_width(&self) -> BusWidth {
        T::supports_bus_width(self)
    }

    fn supports_frequency(&self) -> u32 {
        T::supports_frequency(self)
    }

    fn supports_mmc(&self) -> bool {
        T::supports_mmc(self)
    }
}

/// R1 — Normal status response
///
/// 48-bit, CRC-checked, no busy
pub struct R1 {
    pub status: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CardState {
    Idle,         // 0
    Ready,        // 1
    Ident,        // 2
    Standby,      // 3
    Transfer,     // 4
    Data,         // 5
    Receive,      // 6
    Programming,  // 7
    Reserved(u8), // 8–15
}

impl R1 {
    /// Error bits defined in the SD Physical Spec §4.10.1 (Table 4-41).
    pub const ERR_MASK: u32 = 0xFDF9_8008;

    pub fn to_result(&self) -> Result<(), MmcError> {
        if self.is_error() {
            Err(MmcError::Other)
        } else {
            Ok(())
        }
    }

    pub fn is_error(&self) -> bool {
        self.status & Self::ERR_MASK != 0
    }

    pub fn app_cmd(&self) -> bool {
        self.status & (1 << 5) != 0
    }

    pub fn ready_for_data(&self) -> bool {
        self.status & (1 << 8) != 0
    }

    pub fn current_state(&self) -> CardState {
        let v = ((self.status >> 9) & 0xF) as u8;
        match v {
            0 => CardState::Idle,
            1 => CardState::Ready,
            2 => CardState::Ident,
            3 => CardState::Standby,
            4 => CardState::Transfer,
            5 => CardState::Data,
            6 => CardState::Receive,
            7 => CardState::Programming,
            other => CardState::Reserved(other),
        }
    }
}

impl Response for R1 {
    const CRC: bool = true;
    const LEN: ResponseLen = ResponseLen::R48;
    const BUSY: bool = false;

    fn from_words(buf: &[u32; 4]) -> Self {
        R1 { status: buf[0] }
    }
}

/// R1b — R1 + busy on DAT0
///
/// 48-bit, CRC-checked, *busy*
/// Card holds DAT0 low until internal operation completes.
pub struct R1b {
    pub status: u32,
}

impl Response for R1b {
    const CRC: bool = true;
    const LEN: ResponseLen = ResponseLen::R48;
    const BUSY: bool = true;

    fn from_words(buf: &[u32; 4]) -> Self {
        R1b { status: buf[0] }
    }
}

/// R2 — CID/CSD (136-bit)
///
/// 136-bit, CRC-checked, no busy
pub struct R2 {
    pub words: [u32; 4],
}

impl Response for R2 {
    const CRC: bool = true;
    const LEN: ResponseLen = ResponseLen::R136;
    const BUSY: bool = false;

    fn from_words(buf: &[u32; 4]) -> Self {
        R2 {
            words: [buf[0], buf[1], buf[2], buf[3]],
        }
    }
}

/// R3 — OCR (Operating Conditions)
///
/// 48-bit, *no CRC*, no busy
/// Used during initialization before CRC is enabled.
pub struct R3 {
    pub ocr: u32,
}

impl Response for R3 {
    const CRC: bool = false;
    const LEN: ResponseLen = ResponseLen::R48;
    const BUSY: bool = false;

    fn from_words(buf: &[u32; 4]) -> Self {
        R3 { ocr: buf[0] }
    }
}

/// R6 — Published RCA (SD only)
///
/// 48-bit, CRC-checked, no busy
pub struct R6 {
    pub rca: u16,
    pub status: u16,
}

impl Response for R6 {
    const CRC: bool = true;
    const LEN: ResponseLen = ResponseLen::R48;
    const BUSY: bool = false;

    fn from_words(buf: &[u32; 4]) -> Self {
        let v = buf[0];
        R6 {
            rca: (v >> 16) as u16,
            status: (v & 0xFFFF) as u16,
        }
    }
}

/// R7 — Interface condition (CMD8)
///
/// 48-bit, CRC-checked, no busy
pub struct R7 {
    pub voltage: u8,
    pub check_pattern: u8,
}

impl Response for R7 {
    const CRC: bool = true;
    const LEN: ResponseLen = ResponseLen::R48;
    const BUSY: bool = false;

    fn from_words(buf: &[u32; 4]) -> Self {
        let v = buf[0];
        R7 {
            voltage: ((v >> 8) & 0xF) as u8,
            check_pattern: (v & 0xFF) as u8,
        }
    }
}

/// ===========================================================================
/// SDIO RESPONSES
/// ===========================================================================

/// R4 — SDIO OCR + capability
///
/// 48-bit, *no CRC*, no busy
/// Returned by CMD5 (IO_SEND_OP_COND)
pub struct R4 {
    pub ocr: u32,
}

impl Response for R4 {
    const CRC: bool = false;
    const LEN: ResponseLen = ResponseLen::R48;
    const BUSY: bool = false;

    fn from_words(buf: &[u32; 4]) -> Self {
        R4 { ocr: buf[0] }
    }
}

/// R5 — SDIO Direct I/O response
///
/// 48-bit, CRC-checked, no busy
/// Returned by CMD52 (direct I/O)
pub struct R5 {
    pub flags: u8,
    pub data: u8,
}

impl R5 {
    /// COM_CRC_ERROR: the CRC of the command that triggered this was bad.
    pub const FLAG_COM_CRC_ERROR: u8 = 1 << 7;
    /// ILLEGAL_COMMAND: command not legal in the current state.
    pub const FLAG_ILLEGAL_COMMAND: u8 = 1 << 6;
    /// General ERROR.
    pub const FLAG_ERROR: u8 = 1 << 5;
    /// FUNCTION_NUMBER: requested function does not exist on this card.
    pub const FLAG_FUNCTION_NUMBER: u8 = 1 << 1;
    /// OUT_OF_RANGE: register address out of range for this function.
    pub const FLAG_OUT_OF_RANGE: u8 = 1 << 0;

    /// Mask of all bits that indicate a hard error.
    pub const ERROR_FLAGS: u8 = Self::FLAG_COM_CRC_ERROR
        | Self::FLAG_ILLEGAL_COMMAND
        | Self::FLAG_ERROR
        | Self::FLAG_FUNCTION_NUMBER
        | Self::FLAG_OUT_OF_RANGE;

    pub fn to_result(&self) -> Result<(), MmcError> {
        if self.is_error() {
            Err(MmcError::Other)
        } else {
            Ok(())
        }
    }

    pub fn is_error(&self) -> bool {
        self.flags & Self::ERROR_FLAGS != 0
    }
}

impl Response for R5 {
    const CRC: bool = true;
    const LEN: ResponseLen = ResponseLen::R48;
    const BUSY: bool = false;

    fn from_words(buf: &[u32; 4]) -> Self {
        let v = buf[0];
        R5 {
            flags: ((v >> 8) & 0xFF) as u8,
            data: (v & 0xFF) as u8,
        }
    }
}

/// Bus Tuning Operation
pub trait TuningOp {
    /// Execute the operation. If error, abort the operation and return.
    fn exec<B: MmcBus>(&mut self, bus: &mut B) -> impl Future<Output = Result<bool, MmcError>>;
}

// Allow passing some commands by reference
impl<T: Command> Command for &T {
    const INDEX: u8 = T::INDEX;

    type Resp<'a>
        = T::Resp<'a>
    where
        Self: 'a;

    fn arg(&self) -> u32 {
        T::arg(&self)
    }
}

impl<T: Command> Command for &mut T {
    const INDEX: u8 = T::INDEX;

    type Resp<'a>
        = T::Resp<'a>
    where
        Self: 'a;

    fn arg(&self) -> u32 {
        T::arg(&self)
    }
}

impl<T: ControlCommand> ControlCommand for &T {}

impl<T: BlockCommand> BlockCommand for &mut T {
    fn block_count(&self) -> u32 {
        T::block_count(self)
    }

    fn block_size(&self) -> BlockSize {
        T::block_size(self)
    }
}

impl<T: BlockReadCommand> BlockReadCommand for &mut T {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        T::buf(self)
    }
}

/// Bus Adapter that implements common functionality of all bus users
struct BusAdapter<B: MmcBus, D: DelayNs> {
    pub bus: B,
    pub delay: D,
    pub rca: u16,
}

impl<B: MmcBus, D: DelayNs> BusAdapter<B, D> {
    /// Send the app command notification if this is an app command
    async fn app_cmd(&mut self, app_cmd: bool) -> Result<(), MmcError> {
        if app_cmd {
            self.bus
                .send_command(sd::app_cmd(self.rca))
                .await?
                .to_result()?
        }

        Ok(())
    }

    /// Check whether the card is ready for data
    async fn check_card(&mut self) -> bool {
        if let Ok(status) = self
            .bus
            .send_command(common::card_status(self.rca, false))
            .await
            && CardStatus::<()>::from(status).ready_for_data()
        {
            true
        } else {
            false
        }
    }

    /// Wait for the card to be ready if required
    async fn wait_if_required<R: Response>(&mut self) -> Result<(), MmcError> {
        if !R::BUSY {
            return Ok(());
        }

        // Wait up to 750ms + cmd time for ready after R1b response
        // Note: this is a rather simplistic timeout loop. It can be improved later.
        for _ in 0..750 {
            if self.check_card().await {
                return Ok(());
            }

            self.delay.delay_ms(1).await;
        }

        Err(MmcError::Timeout)
    }

    pub async fn init_idle(&mut self) -> Result<(), MmcError> {
        // While the SD/SDIO card or eMMC is in identification mode,
        // the SDMMC_CK frequency must be no more than 400 kHz.
        self.bus.init_idle(INIT_FREQ).await?;

        // Wait 74 cycles
        self.delay.delay_us(74_000_000 / INIT_FREQ).await;

        self.send_command(common::idle(), false).await?;

        Ok(())
    }

    /// Select one card and place it into the _Tranfer State_
    ///
    /// If `None` is specifed for `card`, all cards are put back into
    /// _Stand-by State_
    pub async fn select_card(&mut self, rca: Option<u16>) -> Result<(), MmcError> {
        match self
            .send_command(common::select_card(rca.unwrap_or(0)), false)
            .await
        {
            Err(MmcError::Timeout) if rca == None => Ok(()),
            result => result.map(|_| ()),
        }
    }

    /// Get the ocr with the provided command
    pub async fn get_ocr<'a, C: ControlCommand + 'a, Ext>(
        &mut self,
        cmd: &'a C,
        app_cmd: bool,
    ) -> Result<OCR<Ext>, MmcError>
    where
        OCR<Ext>: From<<C as Command>::Resp<'a>>,
    {
        // Wait up to 750ms + cmd time for ready after R1b response
        // Note: this is a rather simplistic timeout loop. It can be improved later.
        for _ in 0..750 {
            let ocr: OCR<Ext> = self.send_command(cmd, app_cmd).await?.into();

            if !ocr.is_busy() {
                // Power up done
                return Ok(ocr);
            }

            self.delay.delay_ms(1).await;
        }

        Err(MmcError::Timeout)
    }

    /// Send a command that has no data transfer (e.g., CMD0, CMD8, CMD55).
    ///
    /// Provide `Some(rca)` to execute this as an app cmd.
    pub async fn send_command<'a, C: ControlCommand + 'a>(
        &mut self,
        cmd: C,
        app_cmd: bool,
    ) -> Result<C::Resp<'a>, MmcError> {
        self.app_cmd(app_cmd).await?;
        let res = self.bus.send_command(cmd).await?;
        self.wait_if_required::<C::Resp<'a>>().await?;

        Ok(res)
    }

    /// Read N blocks of fixed size (CMD17, CMD18).
    ///
    /// Provide `Some(rca)` to execute this as an app cmd.
    ///
    /// Do not call this method for CMD53. Instead, call the underlying bus method.
    pub async fn read_blocks<'a, C: BlockReadCommand + 'a>(
        &mut self,
        cmd: C,
        app_cmd: bool,
    ) -> Result<C::Resp<'a>, MmcError> {
        self.app_cmd(app_cmd).await?;
        let res = self.bus.read_blocks(cmd).await?;
        self.wait_if_required::<C::Resp<'a>>().await?;

        Ok(res)
    }

    /// Write N blocks of fixed size (CMD24, CMD25).
    ///
    /// Provide `Some(rca)` to execute this as an app cmd.
    ///
    /// Do not call this method for CMD53. Instead, call the underlying bus method.
    pub async fn write_blocks<'a, C: BlockWriteCommand + 'a>(
        &mut self,
        cmd: C,
        app_cmd: bool,
    ) -> Result<C::Resp<'a>, MmcError> {
        self.app_cmd(app_cmd).await?;
        let res = self.bus.write_blocks(cmd).await?;
        self.wait_if_required::<C::Resp<'a>>().await?;

        Ok(res)
    }
}

/// The signalling scheme used on the SDMMC bus
#[non_exhaustive]
#[allow(missing_docs)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum Signalling {
    #[default]
    SDR12,
    SDR25,
    SDR50,
    SDR104,
    DDR50,
}

/// Represents either an SD or EMMC card
trait Acquirable: Sized + Clone + Default {
    fn acquire<B: MmcBus, D: DelayNs>(
        bus: &mut BusAdapter<B, D>,
        block_size: BlockSize,
        freq: u32,
    ) -> impl Future<Output = Result<Self, MmcError>>;
}

/// Represents either an SD or EMMC card
#[allow(private_bounds)]
pub trait Addressable: Acquirable {
    /// Associated type
    type Ext;

    /// Is this a standard or high capacity peripheral?
    fn get_capacity(&self) -> CardCapacity;

    /// Size in bytes
    fn size(&self) -> u64;

    /// Whether the device supports `CMD23 (SET_BLOCK_COUNT)`.
    fn supports_cmd23(&self) -> bool;

    /// Whether the device supports `ACMD23`.
    fn supports_acmd23(&self) -> bool;
}

/// Represents a block storage device
pub struct BlockDevice<T: Addressable, B: MmcBus, D: DelayNs, const BLOCK_SIZE: usize> {
    info: T,
    bus: BusAdapter<B, D>,
}

/// Card or Emmc storage device
impl<A: Addressable, B: MmcBus, D: DelayNs, const BLOCK_SIZE: usize>
    BlockDevice<A, B, D, BLOCK_SIZE>
{
    /// Create a new block device
    pub async fn new(bus: B, delay: D, freq: u32) -> Result<Self, MmcError> {
        let mut this = Self::new_uninit(bus, delay);
        this.reacquire(freq).await?;

        Ok(this)
    }

    /// Create a new uninit block device
    pub fn new_uninit(bus: B, delay: D) -> Self {
        Self {
            info: A::default(),
            bus: BusAdapter { bus, delay, rca: 0 },
        }
    }

    /// Reacquire the device
    pub async fn reacquire(&mut self, freq: u32) -> Result<(), MmcError> {
        // Clamp the frequency to the supported bus frequency.
        let freq = freq.clamp(0, self.bus.bus.supports_frequency());

        self.bus.init_idle().await?;
        self.info = A::acquire(&mut self.bus, block_size(BLOCK_SIZE), freq).await?;

        Ok(())
    }

    /// Get the card info
    pub fn card(&self) -> A {
        self.info.clone()
    }

    /// Read a data block.
    #[inline]
    async fn read_block(
        &mut self,
        block_idx: u32,
        block: &mut Aligned<A4, [u8; BLOCK_SIZE]>,
    ) -> Result<(), MmcError> {
        let card_capacity = self.info.get_capacity();

        // Always read 1 block of 512 bytes
        // SDSC cards are byte addressed hence the blockaddress is in multiples of 512 bytes
        let addr = match card_capacity {
            CardCapacity::StandardCapacity => block_idx * BLOCK_SIZE as u32,
            _ => block_idx,
        };

        self.bus
            .read_blocks(read_single_block(addr, block), false)
            .await?;

        Ok(())
    }

    /// Read multiple data blocks.
    #[inline]
    async fn read_blocks(
        &mut self,
        block_idx: u32,
        blocks: &mut [Aligned<A4, [u8; BLOCK_SIZE]>],
    ) -> Result<(), MmcError> {
        let card_capacity = self.info.get_capacity();

        // Always read 1 block of 512 bytes
        // SDSC cards are byte addressed hence the blockaddress is in multiples of 512 bytes
        let addr = match card_capacity {
            CardCapacity::StandardCapacity => block_idx * BLOCK_SIZE as u32,
            _ => block_idx,
        };

        self.bus
            .read_blocks(read_multiple_blocks(addr, blocks), false)
            .await?;

        Ok(())
    }

    /// Write a data block.
    #[inline]
    async fn write_block(
        &mut self,
        block_idx: u32,
        block: &Aligned<A4, [u8; BLOCK_SIZE]>,
    ) -> Result<(), MmcError> {
        let card_capacity = self.info.get_capacity();

        // Always read 1 block of 512 bytes
        // SDSC cards are byte addressed hence the blockaddress is in multiples of 512 bytes
        let addr = match card_capacity {
            CardCapacity::StandardCapacity => block_idx * BLOCK_SIZE as u32,
            _ => block_idx,
        };

        self.bus
            .write_blocks(write_single_block(addr, block), false)
            .await?;

        Ok(())
    }

    /// Write multiple data blocks.
    #[inline]
    async fn write_blocks(
        &mut self,
        block_idx: u32,
        blocks: &[Aligned<A4, [u8; BLOCK_SIZE]>],
    ) -> Result<(), MmcError> {
        let card_capacity = self.info.get_capacity();

        // Always read 1 block of 512 bytes
        // SDSC cards are byte addressed hence the blockaddress is in multiples of 512 bytes
        let addr = match card_capacity {
            CardCapacity::StandardCapacity => block_idx * BLOCK_SIZE as u32,
            _ => block_idx,
        };

        if self.info.supports_cmd23() {
            self.bus
                .send_command(sd::set_block_count(blocks.len() as u32), true)
                .await?;
        }

        if self.info.supports_acmd23() {
            self.bus
                .send_command(sd::set_block_count(blocks.len() as u32), true)
                .await?;
        }

        self.bus
            .write_blocks(write_multiple_blocks(addr, blocks), false)
            .await?;

        if !self.info.supports_cmd23() {
            self.bus.send_command(stop_transmission(), false).await?;
        }

        Ok(())
    }
}

impl<A: Addressable, B: MmcBus, D: DelayNs, const BLOCK_SIZE: usize>
    block_device_driver::BlockDevice<BLOCK_SIZE> for BlockDevice<A, B, D, BLOCK_SIZE>
{
    type Align = A4;
    type Error = MmcError;

    #[inline]
    async fn read(
        &mut self,
        block_address: u32,
        blocks: &mut [aligned::Aligned<Self::Align, [u8; BLOCK_SIZE]>],
    ) -> Result<(), Self::Error> {
        assert_eq!(BLOCK_SIZE % 4, 0);

        // TODO: I think block_address needs to be adjusted by the partition start offset
        if blocks.len() == 1 {
            self.read_block(block_address, &mut blocks[0]).await?;
        } else {
            self.read_blocks(block_address, blocks).await?;
        }
        Ok(())
    }

    #[inline]
    async fn write(
        &mut self,
        block_address: u32,
        blocks: &[aligned::Aligned<Self::Align, [u8; BLOCK_SIZE]>],
    ) -> Result<(), Self::Error> {
        assert_eq!(BLOCK_SIZE % 4, 0);

        // TODO: I think block_address needs to be adjusted by the partition start offset
        if blocks.len() == 1 {
            self.write_block(block_address, &blocks[0]).await?;
        } else {
            self.write_blocks(block_address, blocks).await?;
        }
        Ok(())
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        Ok(self.info.size())
    }
}
