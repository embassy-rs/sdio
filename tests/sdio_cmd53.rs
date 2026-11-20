use std::collections::VecDeque;

use aligned::{A4, Aligned};
use embedded_hal_async::delay::DelayNs;
use sdio::sdio::SdioCard;
use sdio::{
    BlockReadCommand, BlockWriteCommand, BusWidth, ByteReadCommand, ByteWriteCommand,
    ControlCommand, MmcBus, MmcError, Response,
};

const BLOCK_SIZE: usize = 64;
const FUNCTION: u8 = 2;
const ADDRESS: u32 = 0x1234;
const GUARD: u8 = 0xa5;

struct Transfer {
    arg: u32,
    data: Vec<u8>,
}

struct MockBus {
    expected: VecDeque<Transfer>,
}

impl MockBus {
    fn new(write: bool, data: &[u8]) -> Self {
        let mut expected = VecDeque::new();
        let block_len = data.len() / BLOCK_SIZE * BLOCK_SIZE;
        for (block_mode, offset, len, count) in [
            (true, 0, block_len, block_len / BLOCK_SIZE),
            (
                false,
                block_len,
                data.len() - block_len,
                data.len() - block_len,
            ),
        ] {
            if len == 0 {
                continue;
            }
            expected.push_back(Transfer {
                arg: (u32::from(write) << 31)
                    | (u32::from(FUNCTION) << 28)
                    | (u32::from(block_mode) << 27)
                    | (1 << 26)
                    | ((ADDRESS + offset as u32) << 9)
                    | count as u32,
                data: data[offset..offset + len].to_vec(),
            });
        }
        Self { expected }
    }

    fn next(&mut self, index: u8, arg: u32) -> Transfer {
        assert_eq!(index, 53);
        let expected = self.expected.pop_front().expect("unexpected transfer");
        assert_eq!(arg, expected.arg, "CMD53 argument");
        expected
    }
}

impl MmcBus for MockBus {
    async fn send_command<'a, C>(&mut self, _cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ControlCommand + 'a,
    {
        panic!("unexpected control command");
    }

    async fn read_blocks<'a, C>(
        &mut self,
        mut cmd: C,
        auto_stop: bool,
    ) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockReadCommand + 'a,
    {
        let expected = self.next(C::INDEX, cmd.arg());
        assert!(!auto_stop);
        assert_eq!(cmd.block_size().len(), BLOCK_SIZE);
        assert_eq!(cmd.block_count() as usize * BLOCK_SIZE, expected.data.len());
        let buf = cmd.buf();
        assert_eq!(buf.len(), expected.data.len());
        buf.copy_from_slice(&expected.data);
        Ok(C::Resp::from_words(&[0; 4]))
    }

    async fn write_blocks<'a, C>(
        &mut self,
        cmd: C,
        auto_stop: bool,
    ) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockWriteCommand + 'a,
    {
        let expected = self.next(C::INDEX, cmd.arg());
        assert!(!auto_stop);
        assert_eq!(cmd.block_size().len(), BLOCK_SIZE);
        assert_eq!(cmd.block_count() as usize * BLOCK_SIZE, expected.data.len());
        assert_eq!(&**cmd.buf(), expected.data);
        Ok(C::Resp::from_words(&[0; 4]))
    }

    async fn read_bytes<'a, C>(&mut self, mut cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ByteReadCommand + 'a,
    {
        let expected = self.next(C::INDEX, cmd.arg());
        assert_eq!(cmd.byte_count(), expected.data.len());
        let buf = cmd.buf();
        assert_eq!(buf.len(), expected.data.len());
        buf.copy_from_slice(&expected.data);
        Ok(C::Resp::from_words(&[0; 4]))
    }

    async fn write_bytes<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ByteWriteCommand + 'a,
    {
        let expected = self.next(C::INDEX, cmd.arg());
        assert_eq!(cmd.byte_count(), expected.data.len());
        assert_eq!(&**cmd.buf(), expected.data);
        Ok(C::Resp::from_words(&[0; 4]))
    }

    async fn init_idle(&mut self, _hz: u32) -> Result<(), MmcError> {
        panic!("unexpected bus initialization");
    }

    fn set_bus(&mut self, _width: BusWidth, _hz: u32) -> Result<(), MmcError> {
        panic!("unexpected bus configuration");
    }
}

struct NoopDelay;

impl DelayNs for NoopDelay {
    async fn delay_ns(&mut self, _ns: u32) {}
}

#[tokio::test]
async fn cmd53_read_uses_slice_length_not_aligned_size() {
    // Include every remainder, exact blocks, and padded lengths around three block boundaries.
    for len in 0..=3 * BLOCK_SIZE + 1 {
        let data: Vec<u8> = (0..len).map(|i| i as u8).collect();
        let mut bus = MockBus::new(false, &data);
        let mut backing = Aligned::<A4, _>([GUARD; 4 * BLOCK_SIZE]);
        let mut card = SdioCard::new_uninit(&mut bus, NoopDelay);
        let aligned: &mut Aligned<A4, [u8]> = &mut backing;
        let buf = &mut aligned[4..4 + len];
        assert_eq!(core::mem::size_of_val(buf), len.next_multiple_of(4));
        card.cmd53_read::<BLOCK_SIZE>(FUNCTION, ADDRESS, buf)
            .await
            .unwrap();
        assert!(bus.expected.is_empty(), "missing transfer for length {len}");
        assert_eq!(&backing[4..4 + len], data, "read length {len}");
        assert!(backing[..4].iter().all(|&b| b == GUARD));
        assert!(backing[4 + len..].iter().all(|&b| b == GUARD));
    }
}

#[tokio::test]
async fn cmd53_write_uses_slice_length_not_aligned_size() {
    for len in 0..=3 * BLOCK_SIZE + 1 {
        let data: Vec<u8> = (0..len).map(|i| i as u8).collect();
        let mut bus = MockBus::new(true, &data);
        let mut backing = Aligned::<A4, _>([GUARD; 4 * BLOCK_SIZE]);
        backing[4..4 + len].copy_from_slice(&data);
        let original = *backing;
        let mut card = SdioCard::new_uninit(&mut bus, NoopDelay);
        let aligned: &Aligned<A4, [u8]> = &backing;
        let buf = &aligned[4..4 + len];
        assert_eq!(core::mem::size_of_val(buf), len.next_multiple_of(4));
        card.cmd53_write::<BLOCK_SIZE>(FUNCTION, ADDRESS, buf)
            .await
            .unwrap();
        assert!(bus.expected.is_empty(), "missing transfer for length {len}");
        assert_eq!(*backing, original, "write modified input for length {len}");
    }
}
