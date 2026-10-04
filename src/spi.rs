//! Contains an `MmcBus` SPI driver.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiBus;

use crate::{
    BlockReadCommand, BlockWriteCommand, BusWidth, ByteReadCommand, ByteWriteCommand, Command,
    ControlCommand, MmcBus, MmcError, Response, ResponseLen,
};

pub trait SetHz {
    fn set_hz(&mut self, hz: u32);
}

/// CRC7 over the 5 command bytes (start+index+arg), poly x^7+x^3+1 (0x89).
fn crc7(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc ^= 0x89;
            }
            crc <<= 1;
        }
    }
    crc >> 1
}

pub struct SpiMmcBus<SPI, CS, DLY> {
    spi: SPI,
    cs: CS,
    delay: DLY,
}

impl<SPI, CS, DLY> SpiMmcBus<SPI, CS, DLY> {
    pub fn new(spi: SPI, cs: CS, delay: DLY) -> Self {
        Self { spi, cs, delay }
    }

    async fn select(&mut self) -> Result<(), MmcError>
    where
        CS: OutputPin,
    {
        self.cs.set_low().map_err(|_| MmcError::Io)
    }

    async fn deselect(&mut self) -> Result<(), MmcError>
    where
        CS: OutputPin,
        SPI: SpiBus<u8>,
    {
        self.cs.set_high().map_err(|_| MmcError::Io)?;
        let _ = self.spi.write(&[0xFF]).await;
        Ok(())
    }

    async fn send_cmd_header<C: Command>(&mut self, cmd: &C) -> Result<(), MmcError>
    where
        SPI: SpiBus<u8>,
        CS: OutputPin,
    {
        self.select().await?;

        let idx = cmd.index() & 0x3F;
        let arg = cmd.arg();

        let mut buf = [
            0x40 | idx,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            0,
        ];
        buf[5] = (crc7(&buf[..5]) << 1) | 1;

        self.spi.write(&buf).await.map_err(|_| MmcError::Io)
    }

    async fn read_r1(&mut self) -> Result<u8, MmcError>
    where
        SPI: SpiBus<u8>,
    {
        let mut b = [0xFF];
        for _ in 0..8 {
            self.spi.read(&mut b).await.map_err(|_| MmcError::Io)?;
            if b[0] != 0xFF {
                return Ok(b[0]);
            }
        }
        Err(MmcError::Timeout)
    }

    async fn wait_not_busy(&mut self) -> Result<(), MmcError>
    where
        SPI: SpiBus<u8>,
    {
        let mut b = [0xFF];
        for _ in 0..65_536 {
            self.spi.read(&mut b).await.map_err(|_| MmcError::Io)?;
            if b[0] == 0xFF {
                return Ok(());
            }
        }
        Err(MmcError::Busy)
    }

    async fn read_response_words<R: Response>(&mut self) -> Result<R, MmcError>
    where
        SPI: SpiBus<u8>,
    {
        let r1 = self.read_r1().await?;

        let total_bytes = match R::LEN {
            ResponseLen::Zero => 0,
            ResponseLen::R48 => 5,
            ResponseLen::R136 => 16,
        };

        let mut raw = [0u8; 1 + 16];
        raw[0] = r1;

        if total_bytes > 0 {
            let mut tmp = [0xFFu8; 16];
            self.spi
                .read(&mut tmp[..total_bytes])
                .await
                .map_err(|_| MmcError::Io)?;
            raw[1..=total_bytes].copy_from_slice(&tmp[..total_bytes]);
        }

        // Skip raw[0]: it holds the R1 status byte, which is not part of the
        // payload. The response parsers expect words to start at the payload.
        let mut words = [0u32; 4];
        for (i, chunk) in raw[1..1 + total_bytes]
            .chunks(4)
            .take(words.len())
            .enumerate()
        {
            let mut w = 0u32;
            for &b in chunk {
                w = (w << 8) | b as u32;
            }
            words[i] = w;
        }

        if R::BUSY {
            self.wait_not_busy().await?;
        }

        Ok(R::from_words(&words))
    }

    async fn read_block(&mut self, buf: &mut [u8]) -> Result<(), MmcError>
    where
        SPI: SpiBus<u8>,
    {
        let mut b = [0xFF];
        for _ in 0..65_536 {
            self.spi.read(&mut b).await.map_err(|_| MmcError::Io)?;
            if b[0] == 0xFE {
                break;
            }
        }
        if b[0] != 0xFE {
            return Err(MmcError::Timeout);
        }

        let mut tmp = [0xFFu8; 512];
        let len = buf.len().min(512);
        self.spi
            .read(&mut tmp[..len])
            .await
            .map_err(|_| MmcError::Io)?;
        buf.copy_from_slice(&tmp[..len]);

        let mut crc = [0xFFu8; 2];
        self.spi.read(&mut crc).await.map_err(|_| MmcError::Io)?;

        Ok(())
    }

    async fn write_block(&mut self, buf: &[u8]) -> Result<(), MmcError>
    where
        SPI: SpiBus<u8>,
    {
        self.spi.write(&[0xFE]).await.map_err(|_| MmcError::Io)?;
        self.spi.write(buf).await.map_err(|_| MmcError::Io)?;
        self.spi
            .write(&[0xFF, 0xFF])
            .await
            .map_err(|_| MmcError::Io)?;

        let mut resp = [0xFF];
        self.spi.read(&mut resp).await.map_err(|_| MmcError::Io)?;
        if (resp[0] & 0x1F) != 0x05 {
            return Err(MmcError::Crc);
        }

        self.wait_not_busy().await
    }
}

impl<SPI, CS, DLY, E> MmcBus for SpiMmcBus<SPI, CS, DLY>
where
    SPI: SpiBus<u8, Error = E> + SetHz,
    CS: OutputPin,
    DLY: DelayNs,
{
    async fn send_command<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ControlCommand + 'a,
    {
        self.send_cmd_header(&cmd).await?;
        let resp = self.read_response_words::<C::Resp<'_>>().await?;
        self.deselect().await?;
        Ok(resp)
    }

    async fn read_blocks<'a, C>(
        &mut self,
        mut cmd: C,
        auto_stop: bool,
    ) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockReadCommand + 'a,
    {
        if auto_stop {
            return Err(MmcError::Unsupported);
        }

        self.send_cmd_header(&cmd).await?;
        let block_size = cmd.block_size().len();
        let total = block_size * cmd.block_count() as usize;
        let slice = &mut cmd.buf()[..total];

        for chunk in slice.chunks_mut(block_size) {
            self.read_block(chunk).await?;
        }

        let resp = self.read_response_words::<C::Resp<'_>>().await?;
        self.deselect().await?;
        Ok(resp)
    }

    async fn write_blocks<'a, C>(
        &mut self,
        cmd: C,
        auto_stop: bool,
    ) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockWriteCommand + 'a,
    {
        if auto_stop {
            return Err(MmcError::Unsupported);
        }

        self.send_cmd_header(&cmd).await?;
        let block_size = cmd.block_size().len();
        let total = block_size * cmd.block_count() as usize;
        let slice = &cmd.buf()[..total];

        for chunk in slice.chunks(block_size) {
            self.write_block(chunk).await?;
        }

        let resp = self.read_response_words::<C::Resp<'_>>().await?;
        self.deselect().await?;
        Ok(resp)
    }

    async fn read_bytes<'a, C>(&mut self, mut cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ByteReadCommand + 'a,
    {
        self.send_cmd_header(&cmd).await?;
        let len = cmd.byte_count();
        let slice = &mut cmd.buf()[..len];

        self.read_block(slice).await?;

        let resp = self.read_response_words::<C::Resp<'_>>().await?;
        self.deselect().await?;
        Ok(resp)
    }

    async fn write_bytes<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: ByteWriteCommand + 'a,
    {
        self.send_cmd_header(&cmd).await?;
        let len = cmd.byte_count();
        let slice = &cmd.buf()[..len];

        self.write_block(slice).await?;

        let resp = self.read_response_words::<C::Resp<'_>>().await?;
        self.deselect().await?;
        Ok(resp)
    }

    async fn init_idle(&mut self, hz: u32) -> Result<(), MmcError> {
        self.spi.set_hz(hz);

        self.cs.set_high().map_err(|_| MmcError::Io)?;
        let dummy = [0xFFu8; 10];
        self.spi.write(&dummy).await.map_err(|_| MmcError::Io)?;
        self.delay.delay_us(1000).await;
        Ok(())
    }

    fn set_bus(&mut self, _width: BusWidth, hz: u32) -> Result<(), MmcError> {
        self.spi.set_hz(hz);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::crc7;
    use super::*;
    use crate::sd::{read_ocr, send_if_cond};
    use core::convert::Infallible;
    use std::collections::VecDeque;

    // Framed command CRC byte sent on the wire = (crc7 << 1) | 1.
    fn framed(bytes: &[u8]) -> u8 {
        (crc7(bytes) << 1) | 1
    }

    #[test]
    fn crc7_matches_known_command_vectors() {
        // SD spec well-known CRCs for these command frames.
        assert_eq!(framed(&[0x40, 0x00, 0x00, 0x00, 0x00]), 0x95); // CMD0
        assert_eq!(framed(&[0x51, 0x00, 0x00, 0x00, 0x00]), 0x55); // CMD17, arg 0
        assert_eq!(framed(&[0x48, 0x00, 0x00, 0x01, 0xAA]), 0x87); // CMD8, arg 0x1AA
    }

    /// SPI mock that serves `rx` bytes on reads (0xFF once exhausted) and
    /// records everything written (commands).
    struct MockSpi {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
    }

    impl embedded_hal::spi::ErrorType for MockSpi {
        type Error = Infallible;
    }

    impl embedded_hal_async::spi::SpiBus<u8> for MockSpi {
        async fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
            for b in buf.iter_mut() {
                *b = self.rx.pop_front().unwrap_or(0xFF);
            }
            Ok(())
        }

        async fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
            self.tx.extend_from_slice(buf);
            Ok(())
        }

        async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            self.write(write).await?;
            self.read(read).await
        }

        async fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.read(buf).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl SetHz for MockSpi {
        fn set_hz(&mut self, _hz: u32) {}
    }

    struct MockCs;

    impl embedded_hal::digital::ErrorType for MockCs {
        type Error = Infallible;
    }

    impl embedded_hal::digital::OutputPin for MockCs {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct MockDelay;

    impl DelayNs for MockDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    fn mock_bus(response: &[u8]) -> SpiMmcBus<MockSpi, MockCs, MockDelay> {
        SpiMmcBus::new(
            MockSpi {
                rx: response.iter().copied().collect(),
                tx: Vec::new(),
            },
            MockCs,
            MockDelay,
        )
    }

    #[tokio::test]
    async fn r7_response_parsing_skips_r1_byte() {
        // Raw bytes captured in https://github.com/embassy-rs/sdio/issues/30:
        // R1 = 0x01 (in idle state), then the 32-bit R7 payload
        // (voltage accepted = 0x1, check pattern = 0xAA), then the CRC byte.
        // The R1 byte must not be packed into the payload word.
        let mut bus = mock_bus(&[0x01, 0x00, 0x00, 0x01, 0xAA, 0xFF]);

        let resp = bus.send_command(send_if_cond(1, 0xAA)).await.unwrap();

        assert_eq!(resp.voltage, 1);
        assert_eq!(resp.check_pattern, 0xAA);
    }

    #[tokio::test]
    async fn r3_response_parsing_skips_r1_byte() {
        // CMD58 (READ_OCR) in SPI mode: R1 = 0x00 (no error), then the 32-bit OCR.
        let mut bus = mock_bus(&[0x00, 0xC0, 0xFF, 0x80, 0x00, 0xFF]);

        let resp = bus.send_command(read_ocr()).await.unwrap();

        assert_eq!(resp.ocr, 0xC0FF8000);
    }
}
