// ============================================================================
// SDIO COMMANDS
// ============================================================================

use core::{fmt, mem, slice};

use aligned::{A4, Aligned};
use block_device_driver::{slice_to_blocks, slice_to_blocks_mut};
use embedded_hal_async::delay::DelayNs;

use crate::{
    BlockCommand, BlockReadCommand, BlockWriteCommand, BusAdapter, BusWidth, ByteCommand,
    ByteReadCommand, ByteWriteCommand, Command, ControlCommand, MmcBus, MmcError, R4, R5,
    sd::{self, BlockSize, OCR, RCA, block_size},
};

/// Type marker for SD-specific extensions.
#[derive(Clone, Copy, Default)]
pub struct SDIO;

/// CMD5 — IO_SEND_OP_COND
pub struct Cmd5 {
    pub switch_to_1_8v_request: bool,
    pub voltage_window: u16,
}
impl Command for Cmd5 {
    const INDEX: u8 = 5;
    type Resp<'a> = R4;
    fn arg(&self) -> u32 {
        u32::from(self.switch_to_1_8v_request) << 24 | u32::from(self.voltage_window & 0x1FF) << 15
    }
}
impl ControlCommand for Cmd5 {}

/// CMD5: IO Op Command
///
/// * `switch_to_1_8v_request` - Switch to 1.8V signaling
/// * `voltage_window` - 9-bit bitfield that represents the voltage window
///   supported by the host. Use 0x1FF to indicate support for the full range of
///   voltages
pub fn io_send_op_cond(switch_to_1_8v_request: bool, voltage_window: u16) -> Cmd5 {
    Cmd5 {
        switch_to_1_8v_request,
        voltage_window,
    }
}

/// CMD52 — IO_RW_DIRECT
pub struct Cmd52 {
    pub write: bool,
    pub function: u8,
    pub raw: bool,
    pub addr: u32,
    pub data: u8,
}
impl Command for Cmd52 {
    const INDEX: u8 = 52;
    type Resp<'a> = R5;
    fn arg(&self) -> u32 {
        let rw = (self.write as u32) << 31;
        let fn_num = (self.function as u32 & 0x7) << 28;
        let raw = (self.raw as u32) << 27;
        let addr = (self.addr & 0x1FFFF) << 9;
        let data = if self.write { self.data as u32 } else { 0 };
        rw | fn_num | raw | addr | data
    }
}
impl ControlCommand for Cmd52 {}

/// CMD53 — IO_RW_EXTENDED (byte mode read)
pub struct Cmd53ByteRead<'a> {
    pub function: u8,
    pub increment: bool,
    pub addr: u32, // 17-bit
    pub buf: &'a mut Aligned<A4, [u8]>,
}

impl<'a> Command for Cmd53ByteRead<'a> {
    const INDEX: u8 = 53;
    type Resp<'b>
        = R5
    where
        Self: 'b;

    fn arg(&self) -> u32 {
        let rw = 0u32 << 31;
        let fnn = (self.function as u32 & 0x7) << 28;
        let blk = 0u32 << 27; // byte mode
        let op = (self.increment as u32) << 26;
        let addr = (self.addr & 0x1FFFF) << 9;
        let cnt = (self.buf.len() as u32) & 0x1FF;
        rw | fnn | blk | op | addr | cnt
    }
}

impl<'a> ByteCommand for Cmd53ByteRead<'a> {
    fn byte_count(&self) -> usize {
        self.buf.len()
    }
}

impl<'a> ByteReadCommand for Cmd53ByteRead<'a> {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        &mut *self.buf
    }
}

/// CMD53 — IO_RW_EXTENDED (byte mode write)
pub struct Cmd53ByteWrite<'a> {
    pub function: u8,
    pub increment: bool,
    pub addr: u32, // 17-bit
    pub buf: &'a Aligned<A4, [u8]>,
}

impl<'a> Command for Cmd53ByteWrite<'a> {
    const INDEX: u8 = 53;
    type Resp<'b>
        = R5
    where
        Self: 'b;

    fn arg(&self) -> u32 {
        let rw = 1u32 << 31;
        let fnn = (self.function as u32 & 0x7) << 28;
        let blk = 0u32 << 27; // byte mode
        let op = (self.increment as u32) << 26;
        let addr = (self.addr & 0x1FFFF) << 9;
        let cnt = (self.buf.len() as u32) & 0x1FF;
        rw | fnn | blk | op | addr | cnt
    }
}

impl<'a> ByteCommand for Cmd53ByteWrite<'a> {
    fn byte_count(&self) -> usize {
        self.buf.len()
    }
}

impl<'a> ByteWriteCommand for Cmd53ByteWrite<'a> {
    fn buf(&self) -> &Aligned<A4, [u8]> {
        self.buf
    }
}

/// CMD53 — IO_RW_EXTENDED (block mode read)
pub struct Cmd53BlockRead<'a, const BLOCK_SIZE: usize> {
    pub function: u8,
    pub increment: bool,
    pub addr: u32, // 17-bit
    pub buf: &'a mut [Aligned<A4, [u8; BLOCK_SIZE]>],
}

impl<'a, const BLOCK_SIZE: usize> Command for Cmd53BlockRead<'a, BLOCK_SIZE> {
    const INDEX: u8 = 53;
    type Resp<'b>
        = R5
    where
        Self: 'b;

    fn arg(&self) -> u32 {
        let rw = 0u32 << 31;
        let fnn = (self.function as u32 & 0x7) << 28;
        let blk = 1u32 << 27; // block mode
        let op = (self.increment as u32) << 26;
        let addr = (self.addr & 0x1FFFF) << 9;
        let cnt = (self.buf.len() as u32) & 0x1FF;
        rw | fnn | blk | op | addr | cnt
    }
}

impl<'a, const BLOCK_SIZE: usize> BlockCommand for Cmd53BlockRead<'a, BLOCK_SIZE> {
    fn block_size(&self) -> BlockSize {
        block_size(BLOCK_SIZE)
    }

    fn block_count(&self) -> u32 {
        self.buf.len() as u32
    }
}

impl<'a, const BLOCK_SIZE: usize> BlockReadCommand for Cmd53BlockRead<'a, BLOCK_SIZE> {
    fn buf(&mut self) -> &mut Aligned<A4, [u8]> {
        unsafe {
            mem::transmute(slice::from_raw_parts_mut(
                self.buf.as_mut_ptr() as *mut _,
                size_of_val(self.buf),
            ))
        }
    }
}

/// CMD53 — IO_RW_EXTENDED (block mode write)
pub struct Cmd53BlockWrite<'a, const BLOCK_SIZE: usize> {
    pub function: u8,
    pub increment: bool,
    pub addr: u32, // 17-bit
    pub buf: &'a [Aligned<A4, [u8; BLOCK_SIZE]>],
}

impl<'a, const BLOCK_SIZE: usize> Command for Cmd53BlockWrite<'a, BLOCK_SIZE> {
    const INDEX: u8 = 53;
    type Resp<'b>
        = R5
    where
        Self: 'b;

    fn arg(&self) -> u32 {
        let rw = 1u32 << 31;
        let fnn = (self.function as u32 & 0x7) << 28;
        let blk = 1u32 << 27; // block mode
        let op = (self.increment as u32) << 26;
        let addr = (self.addr & 0x1FFFF) << 9;
        let cnt = (self.buf.len() as u32) & 0x1FF;
        rw | fnn | blk | op | addr | cnt
    }
}

impl<'a, const BLOCK_SIZE: usize> BlockCommand for Cmd53BlockWrite<'a, BLOCK_SIZE> {
    fn block_size(&self) -> BlockSize {
        block_size(BLOCK_SIZE)
    }

    fn block_count(&self) -> u32 {
        self.buf.len() as u32
    }
}

impl<'a, const BLOCK_SIZE: usize> BlockWriteCommand for Cmd53BlockWrite<'a, BLOCK_SIZE> {
    fn buf(&self) -> &Aligned<A4, [u8]> {
        unsafe {
            mem::transmute(slice::from_raw_parts(
                self.buf.as_ptr() as *const _,
                size_of_val(self.buf),
            ))
        }
    }
}

impl OCR<SDIO> {
    pub fn num_io_functions(&self) -> u8 {
        ((self.0 >> 28) & 0x7) as u8
    }
}

impl fmt::Debug for OCR<SDIO> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("OCR: Operation Conditions Register")
            .field("IO functions", &self.num_io_functions())
            .finish()
    }
}

// SDIO Card Common Control Register (CCCR) and Function Basic Register (FBR)
// address constants, as defined in the SDIO specification §6.9.

// ── CCCR registers (all in function 0 address space) ─────────────────────────

/// CCCR / SDIO revision.
pub const CCCR_REV: u32 = 0x00;
/// SD specification revision supported by the card.
pub const CCCR_SD_REV: u32 = 0x01;
/// I/O enable: bit N enables function N (bits 1–7).
pub const CCCR_IO_ENABLE: u32 = 0x02;
/// I/O ready: read to determine which functions are powered and ready.
pub const CCCR_IO_READY: u32 = 0x03;
/// Interrupt enable: master enable in bit 0; per-function in bits 1–7.
pub const CCCR_INT_ENABLE: u32 = 0x04;
/// Interrupt pending: read-only; indicates which functions have pending IRQs.
pub const CCCR_INT_PENDING: u32 = 0x05;
/// I/O abort: write bit N to abort function N; write bit 3 for card reset.
pub const CCCR_IO_ABORT: u32 = 0x06;
/// Bus interface control (bus width, card detect disable, continuous SPI).
pub const CCCR_BUS_CTRL: u32 = 0x07;
/// Card capability flags.
pub const CCCR_CARD_CAP: u32 = 0x08;
/// Common CIS pointer (3 bytes at 0x09–0x0B).
pub const CCCR_CIS_PTR: u32 = 0x09;
/// Bus suspend / enable function select (SDIO multi-block).
pub const CCCR_BUS_SUSPEND: u32 = 0x0C;
/// Function select.
pub const CCCR_FUNC_SEL: u32 = 0x0D;
/// Exec flags: which functions are currently executing.
pub const CCCR_EXEC_FLAGS: u32 = 0x0E;
/// Ready flags: which functions are ready to execute.
pub const CCCR_READY_FLAGS: u32 = 0x0F;
/// Function 0 block size (low byte; high byte at 0x11).
pub const CCCR_FN0_BLKSZ_LO: u32 = 0x10;
pub const CCCR_FN0_BLKSZ_HI: u32 = 0x11;
/// Power control (SMPC / EMPC bits).
pub const CCCR_POWER_CTRL: u32 = 0x12;
/// Bus speed select / high-speed enable.
pub const CCCR_HIGH_SPEED: u32 = 0x13;

// ── CCCR_BUS_CTRL bit fields ─────────────────────────────────────────────────

/// Bus width field mask (bits [1:0]).
pub const BUS_CTRL_WIDTH_MASK: u8 = 0x03;
/// 1-bit bus width.
pub const BUS_CTRL_WIDTH_1BIT: u8 = 0x00;
/// 4-bit bus width.
pub const BUS_CTRL_WIDTH_4BIT: u8 = 0x02;
/// 8-bit bus width (MMC only).
pub const BUS_CTRL_WIDTH_8BIT: u8 = 0x03;
/// Enable continuous SPI interrupt (SPI mode only).
pub const BUS_CTRL_ECSI: u8 = 0x20;
/// Supports continuous SPI interrupt (SPI mode only).
pub const BUS_CTRL_SCSI: u8 = 0x40;
/// Card detect disable.
pub const BUS_CTRL_CD_DISABLE: u8 = 0x80;

// ── CCCR_CARD_CAP bit fields ──────────────────────────────────────────────────

/// SDC: card supports CMD52 during data transfers.
pub const CARD_CAP_SDC: u8 = 0x01;
/// SMB: card supports multi-block CMD53.
pub const CARD_CAP_SMB: u8 = 0x02;
/// SRW: card supports read-wait.
pub const CARD_CAP_SRW: u8 = 0x04;
/// SBS: card supports bus control.
pub const CARD_CAP_SBS: u8 = 0x08;
/// S4MI: card supports 4-bit multi-block CMD53.
pub const CARD_CAP_S4MI: u8 = 0x10;
/// E4MI: enable 4-bit multi-block CMD53.
pub const CARD_CAP_E4MI: u8 = 0x20;
/// LSC: low-speed card (400 kHz only).
pub const CARD_CAP_LSC: u8 = 0x40;
/// 4BLS: 4-bit LSBL (low-speed card with 4-bit support).
pub const CARD_CAP_4BLS: u8 = 0x80;

// ── CCCR_HIGH_SPEED bit fields ────────────────────────────────────────────────

/// SHS: card supports high-speed mode.
pub const HIGH_SPEED_SHS: u8 = 0x01;
/// EHS: enable high-speed mode (write 1 to activate 50 MHz operation).
pub const HIGH_SPEED_EHS: u8 = 0x02;

// ── Function Basic Registers (FBR) ────────────────────────────────────────────
//
// Each function N has its own FBR block at base address 0x100 * N.

/// FBR offset: standard SDIO function interface code.
pub const FBR_FUNCTION_IF: u32 = 0x00;
/// FBR offset: extended interface code.
pub const FBR_EXT_IF: u32 = 0x01;
/// FBR offset: power selection.
pub const FBR_POWER_SEL: u32 = 0x02;
/// FBR offset: CIS pointer (3 bytes at +0x09 … +0x0B).
pub const FBR_CIS_PTR: u32 = 0x09;
/// FBR offset: CSA (Card Storage Area) pointer (3 bytes at +0x0C … +0x0E).
pub const FBR_CSA_PTR: u32 = 0x0C;
/// FBR offset: CSA data window.
pub const FBR_CSA_DATA: u32 = 0x0F;
/// FBR offset: I/O block size low byte.
pub const FBR_BLKSZ_LO: u32 = 0x10;
/// FBR offset: I/O block size high byte.
pub const FBR_BLKSZ_HI: u32 = 0x11;

/// FBR base for function N (1..7)
#[inline]
pub const fn fbr_base(function: u8) -> u32 {
    (function as u32) * 0x100
}

/// FBR registers (per function)
#[inline]
pub const fn fbr_std_func_if(function: u8) -> u32 {
    fbr_base(function)
}
#[inline]
pub const fn fbr_func_enable(function: u8) -> u32 {
    fbr_base(function) + 0x02
}
#[inline]
pub const fn fbr_block_size_low(function: u8) -> u32 {
    fbr_base(function) + 0x10
}
#[inline]
pub const fn fbr_block_size_high(function: u8) -> u32 {
    fbr_base(function) + 0x11
}

/// SDIO Response
pub struct SdioResponse {
    pub data: u8,
    pub flags: u8,
}

impl From<R5> for SdioResponse {
    fn from(resp: R5) -> Self {
        SdioResponse {
            data: resp.data,
            flags: resp.flags,
        }
    }
}

/// SDIO Interface
pub struct SdioCard<B: MmcBus, D: DelayNs> {
    bus: BusAdapter<B, D>,
    ocr: OCR<SDIO>,
}

impl<B: MmcBus, D: DelayNs> SdioCard<B, D> {
    /// Create a new SDIO card
    pub async fn new(bus: B, delay: D, freq: u32) -> Result<Self, MmcError> {
        let mut this = Self::new_uninit(bus, delay);
        this.reacquire(freq).await?;

        Ok(this)
    }

    /// Create a new uninit block device
    pub fn new_uninit(bus: B, delay: D) -> Self {
        Self {
            bus: BusAdapter { bus, delay, rca: 0 },
            ocr: OCR::default(),
        }
    }

    /// Reacquire the device
    pub async fn reacquire(&mut self, freq: u32) -> Result<(), MmcError> {
        self.acquire(freq).await
    }

    /// Initializes the card into a known state (or at least tries to).
    async fn acquire(&mut self, freq: u32) -> Result<(), MmcError> {
        // Clamp the frequency to the supported bus frequency.
        let freq = freq.clamp(0, self.bus.bus.supports_frequency());

        // Get the bus width configured in the Sdmmc peripheral
        let bus_width = self.bus.bus.supports_bus_width();

        // Go.
        self.bus.init_idle().await?;

        // CMD5 inquiry (arg = 0): the card reports the voltage window it
        // supports but does not begin powering up.
        let inquiry: OCR<SDIO> = self
            .bus
            .send_command(io_send_op_cond(false, 0x0), false)
            .await?
            .into();

        // Power-up: re-issue CMD5 with a non-empty voltage window until the
        // card clears its busy (C) bit. Requesting an empty window (arg = 0)
        // here means the card never powers up, so this would loop until it
        // times out. Fall back to the full range if the inquiry came back
        // empty.
        let mut window = ((inquiry.0 >> 15) & 0x1FF) as u16;
        if window == 0 {
            window = 0x1FF;
        }
        self.ocr = self
            .bus
            .get_ocr(&io_send_op_cond(false, window), false)
            .await?;

        // UDB-based SDIO does not support io volt switch sequence

        // Get RCA
        self.bus.rca = RCA::<SDIO>::from(
            self.bus
                .send_command(sd::send_relative_address(), false)
                .await?,
        )
        .address();

        // Select the card with RCA
        self.bus.select_card(Some(self.bus.rca)).await?;

        let cap = self.cmd52_read(0, CCCR_CARD_CAP).await?;

        // Determine the widest bus the card AND host both support.
        let card_supports_4bit = cap & CARD_CAP_4BLS != 0 || cap & CARD_CAP_LSC == 0;

        let (bus_width_reg, bus_width) = if matches!(bus_width, BusWidth::W4) && card_supports_4bit
        {
            (BUS_CTRL_WIDTH_4BIT, BusWidth::W4)
        } else {
            (BUS_CTRL_WIDTH_1BIT, BusWidth::W1)
        };

        let bus_ctrl = self.cmd52_read(0, CCCR_BUS_CTRL).await?;
        self.cmd52_write(
            0,
            CCCR_BUS_CTRL,
            (bus_ctrl & !BUS_CTRL_WIDTH_MASK) | bus_width_reg,
        )
        .await?;

        // Up to 25Mhz
        self.bus.bus.set_bus(bus_width, freq.clamp(0, 25_000_000))?;

        let hs_reg = self.cmd52_read(0, CCCR_HIGH_SPEED).await?;
        if freq > 25_000_000 && hs_reg & HIGH_SPEED_SHS != 0 {
            self.cmd52_write(0, CCCR_HIGH_SPEED, hs_reg | HIGH_SPEED_EHS)
                .await?;

            // Up to max_f
            self.bus.bus.set_bus(bus_width, freq)?;
        }

        Ok(())
    }

    // ── Function management ───────────────────────────────────────────────────

    /// Enable one or more SDIO functions by writing to CCCR IO_ENABLE.
    ///
    /// `func_mask` is an 8-bit mask where bit N enables function N (bits 1–7).
    /// Blocks until IO_READY confirms the functions are up.
    pub async fn enable_functions(&mut self, func_mask: u8) -> Result<(), MmcError> {
        if func_mask & 0xFE == 0 {
            return Ok(());
        }
        let current = self.cmd52_read(0, CCCR_IO_ENABLE).await?;
        self.cmd52_write(0, CCCR_IO_ENABLE, current | func_mask)
            .await?;

        // Poll IO_READY until all requested functions assert their bits.
        loop {
            let ready = self.cmd52_read(0, CCCR_IO_READY).await?;
            if ready & func_mask == func_mask {
                return Ok(());
            }
            self.bus.delay.delay_ms(2).await;
        }
    }

    /// Disable SDIO functions by clearing their bits in CCCR IO_ENABLE.
    pub async fn disable_functions(&mut self, func_mask: u8) -> Result<(), MmcError> {
        let current = self.cmd52_read(0, CCCR_IO_ENABLE).await?;
        self.cmd52_write(0, CCCR_IO_ENABLE, current & !func_mask)
            .await?;
        Ok(())
    }

    /// Enable interrupt signalling for the given function mask (bits 1–7).
    /// Bit 0 in CCCR_INT_ENABLE is the master IRQ enable; this sets it automatically.
    pub async fn enable_interrupts(&mut self, func_mask: u8) -> Result<(), MmcError> {
        self.cmd52_write(0, CCCR_INT_ENABLE, func_mask | 0x01).await
    }

    /// Configure the I/O block size for a function via its FBR.
    pub async fn set_block_size(&mut self, func: u8, size: u16) -> Result<(), MmcError> {
        let base = fbr_base(func);
        self.cmd52_write(0, base + FBR_BLKSZ_LO, (size & 0xFF) as u8)
            .await?;
        self.cmd52_write(0, base + FBR_BLKSZ_HI, (size >> 8) as u8)
            .await?;
        Ok(())
    }

    /// Wait for SDIO irq
    pub async fn wait_for_event(&mut self) -> Result<(), MmcError> {
        self.bus.bus.wait_for_event().await
    }

    // ── CMD52 helpers (single-byte register access) ───────────────────────────

    /// Read a single byte from a function's register space (CMD52).
    pub async fn cmd52_read(&mut self, func: u8, addr: u32) -> Result<u8, MmcError> {
        let resp = self
            .bus
            .send_command(
                Cmd52 {
                    write: false,
                    function: func,
                    raw: false,
                    addr,
                    data: 0,
                },
                false,
            )
            .await?;

        resp.to_result()?;

        Ok(resp.data)
    }

    /// Write a single byte to a function's register space (CMD52).
    pub async fn cmd52_write(&mut self, func: u8, addr: u32, data: u8) -> Result<(), MmcError> {
        let resp = self
            .bus
            .send_command(
                Cmd52 {
                    write: true,
                    function: func,
                    raw: false,
                    addr,
                    data,
                },
                false,
            )
            .await?;

        resp.to_result()?;

        Ok(())
    }

    /// Write a byte and return the register value *after* the write (CMD52 with RAW=1).
    pub async fn cmd52_write_read(
        &mut self,
        func: u8,
        addr: u32,
        data: u8,
    ) -> Result<u8, MmcError> {
        let resp = self
            .bus
            .send_command(
                Cmd52 {
                    write: true,
                    function: func,
                    raw: true,
                    addr,
                    data,
                },
                false,
            )
            .await?;

        resp.to_result()?;

        Ok(resp.data)
    }

    // ── CMD53 helpers (bulk transfers) ────────────────────────────────────────

    /// Read in block mode using cmd53
    pub async fn cmd53_read_blocks<const BLOCK_SIZE: usize>(
        &mut self,
        function: u8,
        increment: bool,
        addr: u32, // 17-bit
        buf: &mut [Aligned<A4, [u8; BLOCK_SIZE]>],
    ) -> Result<(), MmcError> {
        self.bus
            .bus
            .read_blocks(
                Cmd53BlockRead {
                    function,
                    increment,
                    addr,
                    buf,
                },
                false,
            )
            .await?
            .to_result()
    }

    /// Read in multibyte mode using cmd53
    pub async fn cmd53_read_bytes(
        &mut self,
        function: u8,
        increment: bool,
        addr: u32, // 17-bit
        buf: &mut Aligned<A4, [u8]>,
    ) -> Result<(), MmcError> {
        self.bus
            .bus
            .read_bytes(Cmd53ByteRead {
                function,
                increment,
                addr,
                buf,
            })
            .await?
            .to_result()
    }

    /// Read first in block mode and then in multibyte mode using cmd53. Always increments.
    pub async fn cmd53_read<const BLOCK_SIZE: usize>(
        &mut self,
        func: u8,
        mut addr: u32,
        buf: &mut Aligned<A4, [u8]>,
    ) -> Result<(), MmcError> {
        // Use buf.len() (Deref to [u8]) not size_of_val, which rounds up to 4 bytes.
        let byte_part = buf.len() % BLOCK_SIZE;
        let block_part = buf.len() - byte_part;

        if block_part > 0 {
            self.cmd53_read_blocks(
                func,
                true,
                addr,
                slice_to_blocks_mut::<A4, BLOCK_SIZE>(&mut buf[..block_part]),
            )
            .await?;

            addr += block_part as u32;
        }

        if byte_part > 0 {
            self.cmd53_read_bytes(func, true, addr, &mut buf[block_part..])
                .await?;
        }

        Ok(())
    }

    /// Write in block mode using cmd53
    pub async fn cmd53_write_blocks<const BLOCK_SIZE: usize>(
        &mut self,
        function: u8,
        increment: bool,
        addr: u32, // 17-bit
        buf: &[Aligned<A4, [u8; BLOCK_SIZE]>],
    ) -> Result<(), MmcError> {
        self.bus
            .bus
            .write_blocks(
                Cmd53BlockWrite {
                    function,
                    increment,
                    addr,
                    buf,
                },
                false,
            )
            .await?
            .to_result()
    }

    /// Write in multibyte mode using cmd53
    pub async fn cmd53_write_bytes(
        &mut self,
        function: u8,
        increment: bool,
        addr: u32, // 17-bit
        buf: &Aligned<A4, [u8]>,
    ) -> Result<(), MmcError> {
        self.bus
            .bus
            .write_bytes(Cmd53ByteWrite {
                function,
                increment,
                addr,
                buf,
            })
            .await?
            .to_result()
    }

    /// Write first in block mode and then in multibyte mode using cmd53. Always increments.
    pub async fn cmd53_write<const BLOCK_SIZE: usize>(
        &mut self,
        func: u8,
        mut addr: u32,
        buf: &Aligned<A4, [u8]>,
    ) -> Result<(), MmcError> {
        // Use buf.len() (Deref to [u8]) not size_of_val, which rounds up to 4 bytes.
        let byte_part = buf.len() % BLOCK_SIZE;
        let block_part = buf.len() - byte_part;

        if block_part > 0 {
            self.cmd53_write_blocks(
                func,
                true,
                addr,
                slice_to_blocks::<A4, BLOCK_SIZE>(&buf[..block_part]),
            )
            .await?;

            addr += block_part as u32;
        }

        if byte_part > 0 {
            self.cmd53_write_bytes(func, true, addr, &buf[block_part..])
                .await?;
        }

        Ok(())
    }
}
