# sdio

A no-std library for interfacing with SD cards, EMMC, and/or SDIO cards, using the `MmcBus` trait. Current targeted
support is STM32 and ESP32 peripherals. Other chips can support SD/SDIO functionality by implementing this trait.

A struct within this crate implements `block_device_driver::BlockDevice`, which allows no-std file systems to write
cards that are attached to peripherals that implement `MmcBus`. Currently known file systems that implement support
for this include [embedded-fatfs][1] and [exfat-slim][2].

## Definitions

- **sdio**: A transport protocol that allows streaming data to wifi, bluetooth, and other similar devices.
- **sdio card**: A device that implements the sdio protocol.
- **sd**: A protocol that allows reading from and writing to a device blocks of data that are usually 512 bytes in size.
- **sd card**: A removable card that implements a verison of the sd protocol.
- **emmc card**: A fixed card that implements a version of the sd protocol.

## An [embassy][3] project

This crate is part of the embassy project, designed to improve cross-platform support for native SD/SDIO host
peripherals. It is also intended to be a replacement for the now-abandoned `sdio-host` project, with a standard
implementation of common logic across all devices based on the SDIO standard.

```rust
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
    ///
    /// If called with auto_stop set to true, CMD12 must be issued after completing this command.
    fn write_blocks<'a, C>(
        &mut self,
        cmd: C,
        auto_stop: bool,
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
        op: O,
    ) -> impl Future<Output = Result<(), MmcError>>
    where
        O: TuningOp,
    {
        async { Ok(()) }
    }

    /// Wait for DAT1 to be pulled low.
    fn wait_for_event(&mut self) -> impl Future<Output = Result<(), MmcError>> {
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

    /// Optional: whether the host supports the 'auto stop' feature.
    fn supports_auto_stop(&self) -> bool {
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
```

## Resources

- https://www.sdcard.org/downloads/pls/index.html

[1]: https://github.com/MabezDev/embedded-fatfs
[2]: https://github.com/ninjasource/exfat-slim
[3]: https://github.com/embassy-rs/embassy
