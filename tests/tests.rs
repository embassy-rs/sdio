use core::future::Future;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdio::sd::Card;
use sdio::{
    BlockReadCommand, BlockWriteCommand, BusWidth, ByteReadCommand, ByteWriteCommand,
    ControlCommand, MmcBus, MmcError, Response,
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
struct CardState {
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
}

impl CardState {
    fn new(size_bytes: usize) -> Self {
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

    fn wait_busy_if_needed<R: Response>(state: &mut CardState) -> Result<(), MmcError> {
        if R::BUSY {
            if state.is_busy() {
                return Err(MmcError::Busy);
            }
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

            // Simulate busy behavior
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
                    //
                    // Real behavior:
                    //   • Before ready: bit31 = 0 (busy)
                    //   • After ready:  bit31 = 1 (power-up complete)
                    //
                    // Our dummy card becomes ready immediately.

                    st.idle = false;
                    st.ready = true;

                    // Set the "power-up complete" bit (bit 31)
                    let ocr_ready = st.ocr | (1 << 31);

                    Ok(DummyMmcBus::make_response([ocr_ready, 0, 0, 0]))
                }

                2 => {
                    // CMD2: ALL_SEND_CID
                    Ok(DummyMmcBus::make_response(st.cid))
                }
                3 => {
                    // CMD3: SEND_RELATIVE_ADDR
                    Ok(DummyMmcBus::make_response([st.rca as u32, 0, 0, 0]))
                }
                6 => {
                    // CMD6 — SWITCH_FUNCTION (SD mode)
                    //
                    // Argument layout:
                    // [31] Mode: 0 = query, 1 = switch
                    // [23:20] Function group 1 selection (high-speed lives here)
                    //
                    // Host typically does:
                    //   CMD6(mode=0) → read 64-byte status
                    //   CMD6(mode=1, group1=1) → switch to high-speed

                    let arg = cmd.arg();
                    let mode = (arg >> 31) & 1;
                    let group1 = (arg >> 0) & 0xF;

                    if mode == 1 {
                        // SWITCH mode
                        if group1 == 1 {
                            // High-speed function
                            st.high_speed = true;
                        }
                        // Always succeed for dummy card
                        Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                    } else {
                        // QUERY mode
                        //
                        // The actual 64-byte data block is returned in read_blocks(),
                        // not here. We just return R1.
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
                        return Err(MmcError::SignalingSwitchFailed);
                    }
                    st.set_busy(2);
                    Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                }
                16 => {
                    // CMD16 — SET_BLOCKLEN
                    //
                    // Real SD cards in SD mode always accept this command,
                    // but the block length is fixed to 512 bytes.
                    //
                    // We ignore the argument and return success.
                    Ok(DummyMmcBus::make_response([0, 0, 0, 0]))
                }
                _ => {
                    println!("unsupported cmd: {}", C::INDEX);

                    Err(MmcError::IllegalCommand)
                }
            }
        }
    }

    fn read_blocks<'a, C>(
        &mut self,
        mut cmd: C,
    ) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: BlockReadCommand + 'a,
    {
        let state = self.state.clone();
        async move {
            let mut st = state.lock().unwrap();
            DummyMmcBus::wait_busy_if_needed::<C::Resp<'_>>(&mut st)?;

            // Detect CMD6 by index
            if C::INDEX == 6 {
                let buf = cmd.buf();
                let data = &mut *buf[..];

                // Fill with a realistic SWITCH FUNCTION STATUS structure.
                // 512 bits = 64 bytes.
                // For simplicity, we only set the high-speed support bit.

                for b in data.iter_mut() {
                    *b = 0;
                }

                // Function group 1: high-speed supported + selected
                // Byte offsets per SD spec:
                //   0: max current consumption
                //   1: ...
                //   13: function group 1 info
                //   16: function group 1 busy status
                //   17: function group 1 support bits
                //
                // Mark high-speed supported (bit 1)
                data[17] = 0b0000_0010;

                // If high-speed already enabled, mark it selected
                if st.high_speed {
                    data[13] = 0b0000_0010;
                }

                st.set_busy(1);
                return Ok(DummyMmcBus::make_response([0, 0, 0, 0]));
            }

            let block = cmd.arg() as usize;
            let bs = cmd.block_size() as usize;
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

    fn write_blocks<'a, C>(&mut self, cmd: C) -> impl Future<Output = Result<C::Resp<'a>, MmcError>>
    where
        C: BlockWriteCommand + 'a,
    {
        let state = self.state.clone();
        async move {
            let mut st = state.lock().unwrap();
            DummyMmcBus::wait_busy_if_needed::<C::Resp<'_>>(&mut st)?;

            let block = cmd.arg() as usize;
            let bs = cmd.block_size() as usize;
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

    fn set_bus(&mut self, width: BusWidth, hz: u32) -> impl Future<Output = Result<(), MmcError>> {
        let state = self.state.clone();
        async move {
            let mut st = state.lock().unwrap();
            st.width = width;
            st.freq = hz;
            if hz > 25_000_000 {
                st.high_speed = true;
            }
            Ok(())
        }
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
