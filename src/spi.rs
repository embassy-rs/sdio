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

    fn crc7(mut v: u32) -> u8 {
        let mut crc: u8 = 0;
        for _ in 0..40 {
            crc <<= 1;
            if ((v & 0x8000_0000) != 0) ^ ((crc & 0x80) != 0) {
                crc ^= 0x09;
            }
            v <<= 1;
        }
        (crc << 1) | 1
    }

    async fn send_cmd_header<C: Command>(&mut self, cmd: &C) -> Result<(), MmcError>
    where
        SPI: SpiBus<u8>,
        CS: OutputPin,
    {
        self.select().await?;

        let idx = cmd.index() & 0x3F;
        let arg = cmd.arg();
        let header = ((0x40u32 | idx as u32) << 24) | (arg >> 8);
        let crc = Self::crc7(header);

        let buf = [
            0x40 | idx,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            crc,
        ];

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

        let mut words = [0u32; 4];
        for (i, chunk) in raw[..=total_bytes].chunks(4).enumerate() {
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

    async fn read_blocks<'a, C>(&mut self, mut cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockReadCommand + 'a,
    {
        self.send_cmd_header(&cmd).await?;
        let block_size = cmd.block_size() as usize;
        let total = block_size * cmd.block_count() as usize;
        let slice = &mut cmd.buf()[..total];

        for chunk in slice.chunks_mut(block_size) {
            self.read_block(chunk).await?;
        }

        let resp = self.read_response_words::<C::Resp<'_>>().await?;
        self.deselect().await?;
        Ok(resp)
    }

    async fn write_blocks<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: BlockWriteCommand + 'a,
    {
        self.send_cmd_header(&cmd).await?;
        let block_size = cmd.block_size() as usize;
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

    async fn set_bus(&mut self, _width: BusWidth, hz: u32) -> Result<(), MmcError> {
        self.spi.set_hz(hz);
        Ok(())
    }
}
