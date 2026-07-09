use core::future::Future;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdio::common::{BlockSize, CSD, OCR, RCA};
use sdio::emmc::EMMC;
use sdio::sd::{Card, SD, SDStatus};
use sdio::{
    BlockReadCommand, BlockWriteCommand, BusWidth, ByteReadCommand, ByteWriteCommand, CardError,
    ControlCommand, MmcBus, MmcError, R3, R6, Response,
};

use aligned::Aligned;
use embedded_hal_async::delay::DelayNs;

use block_device_driver::BlockDevice as _;
use sdio::BlockDevice;

const INIT_FREQ: u32 = 400_000;
/// ---------------------------------------------------------------------------
/// Internal SD‑card state
/// ---------------------------------------------------------------------------
#[derive(Debug)]
pub struct CardState {
    _powered: bool,
    idle: bool,
    ready: bool,
    rca: u16,
    ocr: u32,
    cid: [u32; 4],
    csd: [u32; 4],
    selected: bool,
    high_speed: bool,
    width: BusWidth,
    freq: u32,
    storage: Vec<u8>,
    busy_until: Option<Instant>,
    last_set_blocklen: Option<u32>,

    // Virtual registers (not backed by storage)
    scr: [u8; 8],
    sd_status: [u8; 64],
    switch_status: [u8; 64],
}

impl CardState {
    fn new(size_bytes: usize) -> Self {
        let mut switch_status = [0u8; 64];
        // Function group 1: high-speed supported (bit 1 in support bits at byte 17)
        switch_status[17] = 0b0000_0010;

        Self {
            _powered: true,
            idle: true,
            ready: false,
            rca: 0x1234,
            ocr: 0x40FF8000, // Standard SD OCR
            cid: [0x12345678, 0x9ABCDEF0, 0xCAFEBABE, 0xDEADBEEF],
            csd: [0x400E0032, 0x5B590000, 0xEDCBA987, 0x00000000],
            selected: false,
            high_speed: false,
            width: BusWidth::W1,
            freq: INIT_FREQ,
            storage: vec![0xFF; size_bytes],
            busy_until: None,
            last_set_blocklen: None,

            // Minimal but sane defaults
            scr: [0x02, 0x58, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00],
            sd_status: [0u8; 64],
            switch_status,
        }
    }

    fn is_busy(&mut self) -> bool {
        if let Some(t) = self.busy_until {
            if Instant::now() < t {
                return true;
            }
        }
        self.busy_until = None;
        false
    }

    fn set_busy(&mut self, ms: u64) {
        self.busy_until = Some(Instant::now() + Duration::from_millis(ms));
    }
}

/// ---------------------------------------------------------------------------
/// Dummy SD‑card bus implementation
/// ---------------------------------------------------------------------------
pub struct DummyMmcBus {
    state: Arc<Mutex<CardState>>,
}

impl DummyMmcBus {
    pub fn new(size_bytes: usize) -> Self {
        Self {
            state: Arc::new(Mutex::new(CardState::new(size_bytes))),
        }
    }

    /// Clone the shared state handle so tests can inspect/seed the card.
    pub fn state(&self) -> Arc<Mutex<CardState>> {
        self.state.clone()
    }

    fn wait_busy_if_needed<R: Response>(state: &mut CardState) -> Result<(), MmcError> {
        if R::BUSY && state.is_busy() {
            return Err(MmcError::Busy);
        }
        Ok(())
    }

    fn make_response<R: Response>(words: [u32; 4]) -> R {
        R::from_words(&words)
    }
}

/// ---------------------------------------------------------------------------
/// Implement MmcBus
/// ---------------------------------------------------------------------------
impl MmcBus for DummyMmcBus {
    fn send_command<'a, C>(&mut self, cmd: C) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: ControlCommand + 'a,
    {
        let state = self.state.clone();
        async move {
            let mut st = state.lock().unwrap();

            DummyMmcBus::wait_busy_if_needed::<C::Resp<'_>>(&mut st)?;

            match C::INDEX {
                0 => {
                    // CMD0: GO_IDLE_STATE
                    st.idle = true;
                    st.ready = false;
                    st.selected = false;
                    Ok(DummyMmcBus::make_response([0; 4]))
                }
                8 => {
                    // CMD8: SEND_IF_COND
                    let arg = cmd.arg();
                    let check = arg & 0xFFF;
                    Ok(DummyMmcBus::make_response([check, 0, 0, 0]))
                }
                55 => {
                    // CMD55: APP_CMD
                    Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                }
                41 => {
                    // ACMD41 — SD_SEND_OP_COND
                    st.idle = false;
                    st.ready = true;
                    let ocr_ready = st.ocr | (1 << 31);
                    Ok(DummyMmcBus::make_response([ocr_ready, 0, 0, 0]))
                }
                2 => {
                    // CMD2: ALL_SEND_CID
                    Ok(DummyMmcBus::make_response(st.cid))
                }
                3 => {
                    // CMD3: SEND_RELATIVE_ADDR
                    Ok(DummyMmcBus::make_response([(st.rca as u32) << 16, 0, 0, 0]))
                }
                6 => {
                    // CMD6 — SWITCH_FUNCTION (SD mode)
                    let arg = cmd.arg();
                    let mode = (arg >> 31) & 1;
                    let group1 = (arg >> 0) & 0xF;

                    if mode == 1 {
                        // SWITCH mode
                        if group1 == 1 {
                            st.high_speed = true;
                            // Mark high-speed selected in switch_status (byte 13, bit1)
                            st.switch_status[13] |= 0b0000_0010;
                        }
                        Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                    } else {
                        // QUERY mode: data comes from read_blocks(), R1 only here
                        Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                    }
                }
                7 => {
                    // CMD7: SELECT_CARD
                    st.selected = true;
                    Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                }
                9 => {
                    // CMD9: SEND_CSD
                    Ok(DummyMmcBus::make_response(st.csd))
                }
                11 => {
                    // CMD11: VOLTAGE_SWITCH
                    if !st.ready {
                        return Err(MmcError::Signaling);
                    }
                    st.set_busy(2);
                    Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                }
                16 => {
                    // CMD16 — SET_BLOCKLEN
                    st.last_set_blocklen = Some(cmd.arg());
                    Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                }
                _ => {
                    println!("unsupported cmd: {}", C::INDEX);
                    Err(MmcError::Card(CardError::IllegalCommand))
                }
            }
        }
    }

    fn read_blocks<'a, C>(
        &mut self,
        mut cmd: C,
        auto_stop: bool,
    ) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: BlockReadCommand + 'a,
    {
        let state = self.state.clone();
        async move {
            if auto_stop {
                return Err(MmcError::Unsupported);
            }

            let mut st = state.lock().unwrap();
            DummyMmcBus::wait_busy_if_needed::<C::Resp<'_>>(&mut st)?;

            // CMD6 (mode=0) — SWITCH FUNCTION STATUS (64 bytes)
            if C::INDEX == 6 {
                let buf = cmd.buf();
                let data = &mut *buf[..];
                data.copy_from_slice(&st.switch_status);
                st.set_busy(1);
                return Ok(DummyMmcBus::make_response([0, 0, 0, 0]));
            }

            // ACMD51 — SEND_SCR (8 bytes)
            if C::INDEX == 51 {
                let buf = cmd.buf();
                let data = &mut *buf[..];
                data[..8].copy_from_slice(&st.scr);
                st.set_busy(1);
                return Ok(DummyMmcBus::make_response([0, 0, 0, 0]));
            }

            // ACMD13 — SD_STATUS (64 bytes)
            if C::INDEX == 13 {
                let buf = cmd.buf();
                let data = &mut *buf[..];
                data[..64].copy_from_slice(&st.sd_status);
                st.set_busy(1);
                return Ok(DummyMmcBus::make_response([0, 0, 0, 0]));
            }

            // Normal data read from storage
            let block = cmd.arg() as usize;
            let bs = cmd.block_size().len();
            let count = cmd.block_count() as usize;
            let start = block * bs;
            let end = start + bs * count;

            if end > st.storage.len() {
                return Err(MmcError::Io);
            }

            let buf = cmd.buf();
            buf.copy_from_slice(&st.storage[start..end]);

            st.set_busy(1);
            Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
        }
    }

    fn write_blocks<'a, C>(
        &mut self,
        cmd: C,
        auto_stop: bool,
    ) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: BlockWriteCommand + 'a,
    {
        let state = self.state.clone();
        async move {
            if auto_stop {
                return Err(MmcError::Unsupported);
            }

            let mut st = state.lock().unwrap();
            DummyMmcBus::wait_busy_if_needed::<C::Resp<'_>>(&mut st)?;

            let block = cmd.arg() as usize;
            let bs = cmd.block_size().len();
            let count = cmd.block_count() as usize;
            let start = block * bs;
            let end = start + bs * count;

            if end > st.storage.len() {
                return Err(MmcError::Io);
            }

            let buf = cmd.buf();
            st.storage[start..end].copy_from_slice(buf);

            st.set_busy(2);
            Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
        }
    }

    fn read_bytes<'a, C>(
        &mut self,
        mut cmd: C,
    ) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: ByteReadCommand + 'a,
    {
        let state = self.state.clone();
        async move {
            let mut st = state.lock().unwrap();
            DummyMmcBus::wait_busy_if_needed::<C::Resp<'_>>(&mut st)?;

            let addr = cmd.arg() as usize;
            let count = cmd.byte_count();
            let end = addr + count;

            if end > st.storage.len() {
                return Err(MmcError::Io);
            }

            let buf = cmd.buf();
            buf.copy_from_slice(&st.storage[addr..end]);

            st.set_busy(1);
            Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
        }
    }

    fn write_bytes<'a, C>(&mut self, cmd: C) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: ByteWriteCommand + 'a,
    {
        let state = self.state.clone();
        async move {
            let mut st = state.lock().unwrap();
            DummyMmcBus::wait_busy_if_needed::<C::Resp<'_>>(&mut st)?;

            let addr = cmd.arg() as usize;
            let count = cmd.byte_count();
            let end = addr + count;

            if end > st.storage.len() {
                return Err(MmcError::Io);
            }

            let buf = cmd.buf();
            st.storage[addr..end].copy_from_slice(buf);

            st.set_busy(1);
            Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
        }
    }

    fn init_idle(&mut self, hz: u32) -> impl Future<Output = Result<(), MmcError>> {
        let state = self.state.clone();
        async move {
            let mut st = state.lock().unwrap();
            st.freq = hz;
            st.width = BusWidth::W1;
            st.idle = true;
            st.ready = false;
            Ok(())
        }
    }

    fn set_bus(&mut self, width: BusWidth, hz: u32) -> Result<(), MmcError> {
        let state = self.state.clone();
        let mut st = state.lock().unwrap();
        st.width = width;
        st.freq = hz;
        if hz > 25_000_000 {
            st.high_speed = true;
            st.switch_status[13] |= 0b0000_0010;
        }
        Ok(())
    }

    fn supports_mmc(&self) -> bool {
        true
    }
}

struct NoopDelay;
impl DelayNs for NoopDelay {
    async fn delay_ns(&mut self, _ns: u32) {}
}

const BLOCK_SIZE: usize = 512;
const CARD_BYTES: usize = 16 * 1024; // 16 KiB fake card

/// Helper to create a fresh block device
async fn make_device() -> BlockDevice<Card, DummyMmcBus, NoopDelay, BLOCK_SIZE> {
    let bus = DummyMmcBus::new(CARD_BYTES);
    let delay = NoopDelay;
    BlockDevice::new_sd_card(bus, 400_000, delay)
        .await
        .expect("init failed")
}

#[tokio::test]
async fn test_init() {
    let mut dev = make_device().await;
    assert!(dev.size().await.unwrap() > 0);
}

#[tokio::test]
async fn test_read_zeroed_blocks() {
    let mut dev = make_device().await;

    let mut blocks = [Aligned([0u8; BLOCK_SIZE]); 1];
    dev.read(0, &mut blocks).await.unwrap();

    assert!(blocks[0].as_slice().iter().all(|&b| b == 0xFF));
}

// #[tokio::test]
// async fn test_write_and_read_back() {
//     let mut dev = make_device().await;
//
//     // Prepare data
//     let mut write_block = Aligned([0u8; BLOCK_SIZE]);
//     for (i, b) in write_block.as_mut_slice().iter_mut().enumerate() {
//         *b = (i & 0xFF) as u8;
//     }
//
//     // Write block 2
//     dev.write(2, &[write_block]).await.unwrap();
//
//     // Read back
//     let mut read_block = Aligned([0u8; BLOCK_SIZE]);
//     dev.read(2, std::slice::from_mut(&mut read_block))
//         .await
//         .unwrap();
//
//     assert_eq!(write_block.as_slice(), read_block.as_slice());
// }

// #[tokio::test]
// async fn test_multi_block_rw() {
//     let mut dev = make_device().await;
//
//     let write_blocks = [
//         Aligned([1u8; BLOCK_SIZE]),
//         Aligned([2u8; BLOCK_SIZE]),
//         Aligned([3u8; BLOCK_SIZE]),
//     ];
//
//     dev.write(4, &write_blocks).await.unwrap();
//
//     let mut read_blocks = [
//         Aligned([0u8; BLOCK_SIZE]),
//         Aligned([0u8; BLOCK_SIZE]),
//         Aligned([0u8; BLOCK_SIZE]),
//     ];
//
//     dev.read(4, &mut read_blocks).await.unwrap();
//
//     assert_eq!(read_blocks[0].as_slice(), &[1u8; BLOCK_SIZE]);
//     assert_eq!(read_blocks[1].as_slice(), &[2u8; BLOCK_SIZE]);
//     assert_eq!(read_blocks[2].as_slice(), &[3u8; BLOCK_SIZE]);
// }

// #[tokio::test]
// async fn test_size_matches_card() {
//     let mut dev = make_device().await;
//     let size = dev.size().await.unwrap();
//     assert_eq!(size, CARD_BYTES as u64);
// }

#[tokio::test]
async fn test_sd_status_parse() {
    // The first 64 bytes of your ACMD13 response
    let status = SDStatus::from([
        128, 0, 0, 0, 3, 0, 0, 0, 4, 0, 144, 0, 20, 5, 26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
    ]);

    // Bus width: (word 15 >> 30) & 3 = (0x80000000 >> 30) & 3 = 0b10 → W4
    assert!(matches!(status.bus_width(), Some(BusWidth::W4)));

    // Secure mode: bit 29 of word 15 → 0
    assert_eq!(status.secure_mode(), false);

    // SD Memory Card Type: low 16 bits of word 15 → 0x0000
    assert_eq!(status.sd_memory_card_type(), 0);

    // Protected area size: word 14 = 0x03000000 → 50331648 bytes
    assert_eq!(status.protected_area_size(), 0x03000000);

    // Speed class: byte 8 = 4
    assert_eq!(status.speed_class(), 4);

    // Move performance: byte 9 = 0
    assert_eq!(status.move_performance(), 0);

    // AU size: nibble in byte 10 = 0x9 → 9
    assert_eq!(status.allocation_unit_size(), 9);

    // Erase size: bytes 11–12 = 0x00 0x14 → 0x0014 = 20 AU
    assert_eq!(status.erase_size(), 20);

    // Erase timeout: bits 23:18 of word 12 = 1
    assert_eq!(status.erase_timeout(), 1);

    // Video speed class: byte 16 = 0
    assert_eq!(status.video_speed_class(), 0);

    // Application performance class: nibble in byte 22 = 0
    assert_eq!(status.app_perf_class(), 0);

    // Discard support: bit 25 of word 8 = 0
    assert_eq!(status.discard_support(), false);
}

#[tokio::test]
async fn test_out_of_bounds_read() {
    let mut dev = make_device().await;

    let mut block = Aligned([0u8; BLOCK_SIZE]);

    let err = dev
        .read(9999, std::slice::from_mut(&mut block))
        .await
        .unwrap_err();

    assert!(matches!(err, MmcError::Io));
}

// #[tokio::test]
// async fn test_out_of_bounds_write() {
//     let mut dev = make_device().await;
//
//     let block = Aligned([0u8; BLOCK_SIZE]);
//
//     let err = dev.write(9999, &[block]).await.unwrap_err();
//
//     assert!(matches!(err, MmcError::Io));
// }

// ---------------------------------------------------------------------------
// Regression tests for upstream parsing/protocol bug fixes
// ---------------------------------------------------------------------------

#[test]
fn test_rca_from_r6_preserves_address_and_status() {
    // RCA must be `(rca << 16) | status`, not `(rca << 16) & status`.
    let rca: RCA<SD> = R6 {
        rca: 0x0007,
        status: 0x0500,
    }
    .into();
    assert_eq!(rca.address(), 0x0007);
    assert_eq!(rca.status(), 0x0500);
}

#[test]
fn test_block_size_len_is_byte_count_not_discriminant() {
    // The enum tag is not the byte count; only len() returns 512.
    assert_eq!(BlockSize::B512.len(), 512);
    assert_ne!(BlockSize::B512 as usize, 512);
}

#[tokio::test]
async fn test_set_block_length_uses_byte_count() {
    // A 512-byte block read must issue CMD16 with arg 512 (the byte count),
    // not the BlockSize enum discriminant.
    let bus = DummyMmcBus::new(CARD_BYTES);
    let state = bus.state();
    let mut dev = BlockDevice::<Card, _, _, BLOCK_SIZE>::new_sd_card(bus, INIT_FREQ, NoopDelay)
        .await
        .unwrap();

    let mut block = Aligned([0u8; BLOCK_SIZE]);
    dev.read(0, std::slice::from_mut(&mut block)).await.unwrap();

    assert_eq!(
        state.lock().unwrap().last_set_blocklen,
        Some(BLOCK_SIZE as u32)
    );
}

#[test]
fn test_emmc_access_mode_reads_high_bits() {
    // Sector mode = OCR bits [30:29] == 0b10; the precedence bug read bits [1:0].
    let sector: OCR<EMMC> = R3 { ocr: 0x4000_0000 }.into();
    assert_eq!(sector.access_mode(), 0b10);

    let byte: OCR<EMMC> = R3 { ocr: 0x0000_0000 }.into();
    assert_eq!(byte.access_mode(), 0b00);
}

#[test]
fn test_emmc_erase_size_blocks_multiplies() {
    // (erase_grp_size + 1) * (erase_grp_mult + 1); was erroneously a sum.
    let csd: CSD<EMMC> = (((3u128) << 42) | (4u128 << 37)).into();
    assert_eq!(csd.erase_size_blocks(), (3 + 1) * (4 + 1));
}

#[tokio::test]
async fn test_sd_status_erase_size_combines_bytes() {
    // ERASE_SIZE is a 16-bit field split across two status bytes; the high byte
    // must be shifted up, not OR-ed onto the low byte.
    let bus = DummyMmcBus::new(CARD_BYTES);
    let state = bus.state();

    {
        let mut st = state.lock().unwrap();

        // According to SD Status layout:
        //   ERASE_SIZE low byte  = sd_status[11]
        //   ERASE_SIZE high byte = sd_status[12]
        st.sd_status[11] = 0x12; // low byte
        st.sd_status[12] = 0x34; // high byte
    }

    let dev = BlockDevice::<Card, _, _, BLOCK_SIZE>::new_sd_card(bus, INIT_FREQ, NoopDelay)
        .await
        .unwrap();

    assert_eq!(dev.card().status.erase_size(), 0x1234);
}

#[test]
fn test_fbr_base_maps_function_to_its_own_block() {
    // FBR for function N lives at 0x100 * N (FN1 = 0x100 ... FN7 = 0x700).
    // The bug `0x100 + N * 0x100` shifted every access up one function, so
    // FN1's block size was written into FN2's FBR.
    use sdio::sdio::{
        FBR_BLKSZ_HI, FBR_BLKSZ_LO, fbr_base, fbr_block_size_high, fbr_block_size_low,
    };

    assert_eq!(fbr_base(1), 0x100);
    assert_eq!(fbr_base(2), 0x200);
    assert_eq!(fbr_base(7), 0x700);

    // Derived helpers must land inside the same function's block.
    assert_eq!(fbr_block_size_low(1), 0x100 + FBR_BLKSZ_LO);
    assert_eq!(fbr_block_size_high(1), 0x100 + FBR_BLKSZ_HI);

    // FN1's block-size register must never collide with FN2's base.
    assert!(fbr_block_size_high(1) < fbr_base(2));
}

// ---------------------------------------------------------------------------
// SD Status (SSR) field decoding — PLSS v7_10 §4.10.2, Table 4-44.
//
// The SSR is 512 bits, transmitted MSB first, so byte 0 = bits [511:504] and
// byte b = bits [511-8b : 504-8b].
// ---------------------------------------------------------------------------
#[test]
fn test_sd_status_decodes_all_fields() {
    let mut raw = [0u8; 64];

    raw[0] = 0xA0; // DAT_BUS_WIDTH=0b10 (4-bit) [511:510], SECURED_MODE=1 [509]
    raw[2] = 0x12; // SD_CARD_TYPE [495:480] = 0x1234
    raw[3] = 0x34;
    raw[4] = 0xDE; // SIZE_OF_PROTECTED_AREA [479:448] = 0xDEADBEEF
    raw[5] = 0xAD;
    raw[6] = 0xBE;
    raw[7] = 0xEF;
    raw[8] = 0x04; // SPEED_CLASS [447:440] (Class 10)
    raw[9] = 0x10; // PERFORMANCE_MOVE [439:432] = 16 MB/s
    raw[10] = 0x90; // AU_SIZE [431:428] = 9 (top nibble of byte 10)
    raw[11] = 0x12; // ERASE_SIZE [423:408] = 0x1234
    raw[12] = 0x34;
    raw[13] = 0x30; // ERASE_TIMEOUT [407:402] = 0x0C (top 6 bits of byte 13)
    raw[15] = 0x1E; // VIDEO_SPEED_CLASS [391:384] = V30
    raw[21] = 0x02; // APP_PERF_CLASS [339:336] = A2 (low nibble of byte 21)
    raw[24] = 0x02; // DISCARD_SUPPORT [313] = bit 1 of byte 24

    let ssr = SDStatus::from(raw);

    assert!(matches!(ssr.bus_width(), Some(BusWidth::W4)));
    assert!(ssr.secure_mode());
    assert_eq!(ssr.sd_memory_card_type(), 0x1234);
    assert_eq!(ssr.protected_area_size(), 0xDEAD_BEEF);
    assert_eq!(ssr.speed_class(), 0x04);
    assert_eq!(ssr.move_performance(), 0x10);
    assert_eq!(ssr.allocation_unit_size(), 0x9);
    assert_eq!(ssr.erase_size(), 0x1234);
    assert_eq!(ssr.erase_timeout(), 0x0C);
    assert_eq!(ssr.video_speed_class(), 0x1E);
    assert_eq!(ssr.app_perf_class(), 0x2);
    assert!(ssr.discard_support());
}

// ---------------------------------------------------------------------------
// SCR field decoding — PLSS v7_10 §5.6, Table 5-17.
//
// SCR is 64 bits, bit 63 = byte 0 bit 7.
// ---------------------------------------------------------------------------
#[test]
fn test_scr_decodes_all_fields() {
    use sdio::sd::{SCR, SDEraseDataStatus, SDSecurity, SDSpecVersion};

    //   SD_SPEC       [59:56] = 2, SD_SPEC3 [47] = 1 -> SDSpecVersion::V3
    //   DATA_STAT_AFTER_ERASE [55] = 0
    //   SD_SECURITY   [54:52] = 4 (SDXC)              -> byte1 = 0b0_100_0101 = 0x45
    //   SD_BUS_WIDTHS [51:48] = 5 (1-bit + 4-bit)
    //   CMD_SUPPORT   [35:32] = 0b1111 (CMD20/23/48/49/58/59) -> byte3 = 0x0F
    let scr = SCR::from([0x02, 0x45, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x00]);

    assert_eq!(scr.version(), SDSpecVersion::V3);
    assert_eq!(scr.security(), SDSecurity::Unknown(4));
    assert_eq!(scr.data_after_erase(), SDEraseDataStatus::Zero);
    assert_eq!(scr.bus_widths(), 0x5);
    assert!(scr.bus_width_one());
    assert!(scr.bus_width_four());
    assert!(scr.supports_cmd20());
    assert!(scr.supports_cmd23());
    assert!(scr.supports_cmd48());
    assert!(scr.supports_cmd49());
}

// ---------------------------------------------------------------------------
// EXT_CSD field decoding — JEDEC JESD84-B51 §7.4.
//
// EXT_CSD is a 512-byte register; each field lives at a fixed byte offset.
// Multi-byte fields (e.g. SEC_COUNT) are little-endian.
// ---------------------------------------------------------------------------
#[test]
fn test_ext_csd_decodes_all_fields() {
    use sdio::emmc::ExtCSD;

    let mut raw = [0u8; 512];

    raw[16] = 0x21; // SECURE_REMOVAL_TYPE
    raw[61] = 0x01; // DATA_SECTOR_SIZE
    raw[192] = 0x08; // EXT_CSD_REV
    raw[194] = 0x02; // CSD_STRUCTURE
    raw[196] = 0x57; // CARD_TYPE
    raw[197] = 0x1F; // DRIVER_STRENGTH
    raw[212] = 0x78; // SEC_COUNT [215:212], little-endian = 0x12345678
    raw[213] = 0x56;
    raw[214] = 0x34;
    raw[215] = 0x12;
    raw[216] = 0x0A; // SLEEP_NOTIFICATION_TIME
    raw[217] = 0x13; // S_A_TIMEOUT
    raw[228] = 0x07; // BOOT_INFO

    let ext = ExtCSD {
        inner: Aligned(raw),
    };

    assert_eq!(ext.secure_removal_type(), 0x21);
    assert_eq!(ext.data_sector_size(), 0x01);
    assert_eq!(ext.extended_csd_revision(), 0x08);
    assert_eq!(ext.csd_structure_version(), 0x02);
    assert_eq!(ext.card_type(), 0x57);
    assert_eq!(ext.driver_strength(), 0x1F);
    assert_eq!(ext.sector_count(), 0x1234_5678);
    assert_eq!(ext.sleep_notification_time(), 0x0A);
    assert_eq!(ext.sleep_awake_timeout(), 0x13);
    assert_eq!(ext.boot_info(), 0x07);
}
