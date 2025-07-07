#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

#[derive(Debug, Clone, Copy)]
pub enum Error {
    SpiWriteFailed,
    SpiReadFailed,
    CSLowfailed,
    CSHighfailed,
}

#[cfg(not(feature = "manual_delay"))]
pub struct W25QXXXJV<'a, SPI: SpiBus, CS: OutputPin, D: DelayNs> {
    spi: SPI,
    cs: CS,
    model: Model,
    spispeed: SpiSpeed,
    delay: &'a mut D,
}

pub struct W25QXXXJV<SPI: SpiBus, CS: OutputPin> {
    spi: SPI,
    cs: CS,
    model: Model,
    spispeed: SpiSpeed,
}

#[cfg(not(feature = "manual_delay"))]
impl<'a, SPI: SpiBus, CS: OutputPin, D: DelayNs> W25QXXXJV<'a, SPI, CS, D> {
    fn cs_high(&mut self) -> Result<(), Error> {
        self.cs.set_high().map_err(|_| Error::CSHighfailed)
    }
    fn cs_low(&mut self) -> Result<(), Error> {
        self.cs.set_low().map_err(|_| Error::CSLowfailed)
    }
    fn spi_read(&mut self, read: &mut [u8]) -> Result<(), Error> {
        self.spi.read(read).map_err(|_| Error::SpiReadFailed)
    }
    fn spi_write(&mut self, write: &[u8]) -> Result<(), Error> {
        self.spi.write(write).map_err(|_| Error::SpiWriteFailed)
    }
    pub fn new(
        spi: SPI,
        cs: CS,
        spispeed: SpiSpeed,
        model: Model,
        delay: &'a mut D,
    ) -> Result<Self, Error> {
        let mut w25q = Self {
            spi,
            cs,
            model,
            spispeed,
            delay,
        };
        w25q.cs_high()?;
        Ok(w25q)
    }
    pub fn destroy(self) -> SPI {
        self.spi
    }

    pub fn change_model(&mut self, model: Model) {
        self.model = model
    }

    pub fn change_spispeed(&mut self, spispeed: SpiSpeed) {
        self.spispeed = spispeed
    }

    fn write_enable(&mut self) -> Result<(), Error> {
        self.cs_low()?;
        self.spi_write(&[0x06])?;
        self.cs_high()?;
        self.delay.delay_ms(5);
        Ok(())
    }

    // WARN: Only work in Simple SPI not Dual/Quad IO.
    pub fn manufacture_device_id(&mut self) -> Result<[u8; 2], Error> {
        let mut buffer = [0u8; 2];

        self.cs_low()?;
        match self.spispeed {
            SpiSpeed::Single | SpiSpeed::Fast => {
                self.spi_write(&[0x90, 0, 0, 0])?;
            }
            _ => return Ok(buffer),
        }
        self.spi_read(&mut buffer)?;
        self.cs_high()?;

        Ok(buffer)
    }

    pub fn read_unique_id(&mut self) -> Result<u64, Error> {
        let mut buffer = [0u8; 8];
        self.cs_low()?;
        self.spi_write(&[0x4b, 0, 0, 0, 0])?;
        self.spi_read(&mut buffer)?;
        self.cs_high()?;
        Ok(u64::from_be_bytes(buffer))
    }

    pub fn read_data(&mut self, addr: u32, read: &mut [u8]) -> Result<(), Error> {
        let mut addrbyte = ((!0 >> 8) & addr).to_be_bytes();

        if read.is_empty() {
            return Ok(());
        }

        use SpiSpeed::*;
        addrbyte[0] = self.spispeed as u8;

        self.cs_low()?;
        self.spi_write(&addrbyte)?;
        match self.spispeed {
            Single => {
                self.spi_read(read)?;
            }
            Fast => {
                // Dummy clocks
                self.spi_read(&mut [0])?;
                // actual read
                self.spi_read(read)?;
            }
            FastDual => {
                // Dummy clocks
                self.spi_read(&mut [0, 0])?;
                // actual read
                self.spi_read(read)?;
            }
            FastQuad => {
                // Dummy clocks
                self.spi_read(&mut [0, 0, 0, 0])?;
                // actual read
                self.spi_read(read)?;
            }
        }

        self.cs_high()?;
        Ok(())
    }

    /**
    From Status Register-1 (S7-S0), the block protect bit is S2,S3,S4 called BP0,BP1,BP2
    SEC (sector protect) and TB (Top/Buttom protect) is on S6 and S5 respectivily
    */
    pub fn protect_portion(
        &mut self,
        portion: Portion,
        part: Part,
        complement: bool,
    ) -> Result<(), Error> {
        self.write_enable()?;

        let mut bufstatus1 = 0;
        let mut bufstatus2 = 0;

        self.cs_low()?;
        self.spi_write(&[0x05])?;
        self.spi_read(&mut [bufstatus1])?;
        self.cs_high()?;

        self.cs_low()?;
        self.spi_write(&[0x35])?;
        self.spi_read(&mut [bufstatus2])?;
        self.cs_high()?;

        bufstatus1 &= !0b1111100;

        // BP2,BP1,BP0 = S4,S3,S2
        match part {
            Part::Zero => {
                // 000
            }
            Part::Frac64 => {
                // 001
                bufstatus1 |= 0b00100;
            }
            Part::Frac32 => {
                // 010
                bufstatus1 |= 0b01000;
            }
            Part::Frac16 => {
                // 011
                bufstatus1 |= 0b01100;
            }
            Part::Frac8 => {
                // 100
                bufstatus1 |= 0b10000;
            }
            Part::Frac4 => {
                // 101
                bufstatus1 |= 0b10100;
            }
            Part::Frac2 => {
                // 110
                bufstatus1 |= 0b11000;
            }
            Part::Full => {
                // 111
                bufstatus1 |= 0b11100
            }
            // SEC = 1, S6 = 1 for sector protect
            Part::Frac1024 => {
                // 001
                bufstatus1 |= 0b1000100;
            }
            Part::Frac512 => {
                // 010
                bufstatus1 |= 0b1001000;
            }
            Part::Frac256 => {
                // 011
                bufstatus1 |= 0b1001100;
            }
            Part::Frac128 => {
                // 100
                bufstatus1 |= 0b1010000;
            }
        }

        // TB = 1, S5 = 1
        if let Portion::Lower = portion {
            bufstatus1 |= 0b100000
        }
        if complement {
            bufstatus2 |= 0b01000000
        }

        self.cs_low()?;
        self.spi_write(&[0x01, bufstatus1, bufstatus2])?;
        self.cs_high()?;

        Ok(())
    }

    pub fn global_block_lock(&mut self) -> Result<(), Error> {
        self.cs_low()?;
        self.spi_write(&[0x7e])?;
        self.cs_high()?;
        Ok(())
    }
    pub fn global_block_unlock(&mut self) -> Result<(), Error> {
        self.cs_low()?;
        self.spi_write(&[0x98])?;
        self.cs_high()?;
        Ok(())
    }

    /// WARN: It takes 45ms to erase entire block.
    pub fn erase_sector(&mut self, addr: u32) -> Result<(), Error> {
        self.write_enable()?;
        if addr
            > match self.model {
                Model::Q128 => 16777216,
                Model::Q64 => 8388608,
                Model::Q32 => 4194304,
                Model::NotSure => 16777216,
            }
        {
            return Ok(());
        }
        let addr = 4096 * (addr / 4096); // make sure erase from starting address
        let mut addrbyte = ((!0 >> 8) & addr).to_be_bytes();
        addrbyte[0] = 0x20;

        // send the erase command
        self.cs_low()?;
        self.spi_write(&addrbyte)?;
        self.cs_high()?;

        self.delay.delay_ms(45);
        // wait for erasing, it may take long so it need to check the busy bit
        let mut status = [0u8];

        loop {
            self.cs_low()?;
            self.spi_write(&[0x05])?;
            self.spi_read(&mut status)?;
            self.cs_high()?;

            if status[0] & 0b1 == 0 {
                break;
            }
        }

        Ok(())
    }

    /// WARN: It takes 150ms to erase entire block.
    pub fn erase_halfblock(&mut self, addr: u32) -> Result<(), Error> {
        self.write_enable()?;
        if addr
            > match self.model {
                Model::Q128 => 16777216,
                Model::Q64 => 8388608,
                Model::Q32 => 4194304,
                Model::NotSure => 16777216,
            }
        {
            return Ok(());
        }
        let addr = 32768 * (addr / 32768); // make sure erase from starting address
        let mut addrbyte = ((!0 >> 8) & addr).to_be_bytes();
        addrbyte[0] = 0x52;

        // send the erase command
        self.cs_low()?;
        self.spi_write(&addrbyte)?;
        self.cs_high()?;

        self.delay.delay_ms(120);
        // wait for erasing, it may take long so it need to check the busy bit
        let mut status = [0u8];

        loop {
            self.cs_low()?;
            self.spi_write(&[0x05])?;
            self.spi_read(&mut status)?;
            self.cs_high()?;

            if status[0] & 0b1 == 0 {
                break;
            }
        }

        Ok(())
    }
    /// WARN: It takes >150ms to erase entire block.
    pub fn erase_block(&mut self, addr: u32) -> Result<(), Error> {
        if addr
            > match self.model {
                Model::Q128 => 16777216,
                Model::Q64 => 8388608,
                Model::Q32 => 4194304,
                Model::NotSure => 16777216,
            }
        {
            return Ok(());
        }
        let addr = 65536 * (addr / 65536); // make sure erase from starting address
        let mut addrbyte = ((!0 >> 8) & addr).to_be_bytes();
        addrbyte[0] = 0xd8;

        self.write_enable()?;
        // send the erase command
        self.cs_low()?;
        self.spi_write(&addrbyte)?;
        self.cs_high()?;

        self.delay.delay_ms(200);

        // wait for erasing, it may take long so it need to check the busy bit
        let mut status = [0u8];

        loop {
            self.cs_low()?;
            self.spi_write(&[0x05])?;
            self.spi_read(&mut status)?;
            self.cs_high()?;

            if status[0] & 0b1 == 0 {
                break;
            }
        }

        Ok(())
    }
    /// WARN: It takes 40S to erase entire block.
    pub fn erase_chip(&mut self) -> Result<(), Error> {
        self.write_enable()?;
        // send the erase command
        self.cs_low()?;
        self.spi_write(&[0xc7])?;
        self.cs_high()?;

        self.delay.delay_ms(40000);

        // wait for erasing, it may take long so it need to check the busy bit
        let mut status = [0u8];

        loop {
            self.cs_low()?;
            self.spi_write(&[0x05])?;
            self.spi_read(&mut status)?;
            self.cs_high()?;

            if status[0] & 0b1 == 0 {
                break;
            }
        }

        Ok(())
    }

    pub fn write_program(&mut self, addr: u32, write: &[u8]) -> Result<(), Error> {
        if write.is_empty() {
            return Ok(());
        }

        // Erase sectors first
        let mut start_sector = addr / 4096;
        let end_sector = (addr + write.len() as u32 - 1) / 4096;

        while start_sector <= end_sector {
            if start_sector % 16 == 0 && start_sector + 16 <= end_sector {
                self.erase_block(start_sector * 4096)?;
                start_sector += 16;
            } else if start_sector % 8 == 0 && start_sector + 8 <= end_sector {
                self.erase_halfblock(start_sector * 4096)?;
                start_sector += 8;
            } else {
                self.erase_sector(start_sector * 4096)?;
                start_sector += 1;
            }
        }

        // Write data in page-sized chunks
        const PAGE_SIZE: usize = 256;
        let mut current_addr = addr;
        let mut remaining_data = write;

        while !remaining_data.is_empty() {
            let page_offset = (current_addr % PAGE_SIZE as u32) as usize;
            let bytes_to_write = core::cmp::min(PAGE_SIZE - page_offset, remaining_data.len());

            let mut addrbyte = ((!0 >> 8) & current_addr).to_be_bytes();
            addrbyte[0] = 0x02;

            self.write_enable()?;
            self.cs_low()?;
            self.spi_write(&addrbyte)?;
            self.spi_write(&remaining_data[..bytes_to_write])?;
            self.cs_high()?;

            self.delay.delay_ms(5);

            // Wait for write completion
            let mut status = [0u8];
            loop {
                self.cs_low()?;
                self.spi_write(&[0x05])?;
                self.spi_read(&mut status)?;
                self.cs_high()?;

                if status[0] & 0b1 == 0 {
                    break;
                }
            }

            current_addr += bytes_to_write as u32;
            remaining_data = &remaining_data[bytes_to_write..];
        }

        Ok(())
    }
}

impl<SPI: SpiBus, CS: OutputPin> W25QXXXJV<SPI, CS> {
    fn cs_high(&mut self) -> Result<(), Error> {
        self.cs.set_high().map_err(|_| Error::CSHighfailed)
    }
    fn cs_low(&mut self) -> Result<(), Error> {
        self.cs.set_low().map_err(|_| Error::CSLowfailed)
    }
    fn spi_read(&mut self, read: &mut [u8]) -> Result<(), Error> {
        self.spi.read(read).map_err(|_| Error::SpiReadFailed)
    }
    fn spi_write(&mut self, write: &[u8]) -> Result<(), Error> {
        self.spi.write(write).map_err(|_| Error::SpiWriteFailed)
    }
    pub fn new(spi: SPI, cs: CS, spispeed: SpiSpeed, model: Model) -> Result<Self, Error> {
        let mut w25q = Self {
            spi,
            cs,
            model,
            spispeed,
        };
        w25q.cs_high()?;
        Ok(w25q)
    }
    pub fn destroy(self) -> SPI {
        self.spi
    }

    pub fn change_model(&mut self, model: Model) {
        self.model = model
    }

    pub fn change_spispeed(&mut self, spispeed: SpiSpeed) {
        self.spispeed = spispeed
    }

    fn write_enable<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.cs_low()?;
        self.spi_write(&[0x06])?;
        self.cs_high()?;
        delay.delay_ms(5);
        Ok(())
    }

    // WARN: Only work in Simple SPI not Dual/Quad IO.
    pub fn manufacture_device_id(&mut self) -> Result<[u8; 2], Error> {
        let mut buffer = [0u8; 2];

        self.cs_low()?;
        match self.spispeed {
            SpiSpeed::Single | SpiSpeed::Fast => {
                self.spi_write(&[0x90, 0, 0, 0])?;
            }
            _ => return Ok(buffer),
        }
        self.spi_read(&mut buffer)?;
        self.cs_high()?;

        Ok(buffer)
    }

    pub fn read_unique_id(&mut self) -> Result<u64, Error> {
        let mut buffer = [0u8; 8];
        self.cs_low()?;
        self.spi_write(&[0x4b, 0, 0, 0, 0])?;
        self.spi_read(&mut buffer)?;
        self.cs_high()?;
        Ok(u64::from_be_bytes(buffer))
    }

    pub fn read_data(&mut self, addr: u32, read: &mut [u8]) -> Result<(), Error> {
        let mut addrbyte = ((!0 >> 8) & addr).to_be_bytes();

        if read.is_empty() {
            return Ok(());
        }

        use SpiSpeed::*;
        addrbyte[0] = self.spispeed as u8;

        self.cs_low()?;
        self.spi_write(&addrbyte)?;
        match self.spispeed {
            Single => {
                self.spi_read(read)?;
            }
            Fast => {
                // Dummy clocks
                self.spi_read(&mut [0])?;
                // actual read
                self.spi_read(read)?;
            }
            FastDual => {
                // Dummy clocks
                self.spi_read(&mut [0, 0])?;
                // actual read
                self.spi_read(read)?;
            }
            FastQuad => {
                // Dummy clocks
                self.spi_read(&mut [0, 0, 0, 0])?;
                // actual read
                self.spi_read(read)?;
            }
        }

        self.cs_high()?;
        Ok(())
    }

    /**
    From Status Register-1 (S7-S0), the block protect bit is S2,S3,S4 called BP0,BP1,BP2
    SEC (sector protect) and TB (Top/Buttom protect) is on S6 and S5 respectivily
    */
    pub fn protect_portion<D: DelayNs>(
        &mut self,
        portion: Portion,
        part: Part,
        complement: bool,
        delay: &mut D,
    ) -> Result<(), Error> {
        self.write_enable(delay)?;

        let mut bufstatus1 = 0;
        let mut bufstatus2 = 0;

        self.cs_low()?;
        self.spi_write(&[0x05])?;
        self.spi_read(&mut [bufstatus1])?;
        self.cs_high()?;

        self.cs_low()?;
        self.spi_write(&[0x35])?;
        self.spi_read(&mut [bufstatus2])?;
        self.cs_high()?;

        bufstatus1 &= !0b1111100;

        // BP2,BP1,BP0 = S4,S3,S2
        match part {
            Part::Zero => {
                // 000
            }
            Part::Frac64 => {
                // 001
                bufstatus1 |= 0b00100;
            }
            Part::Frac32 => {
                // 010
                bufstatus1 |= 0b01000;
            }
            Part::Frac16 => {
                // 011
                bufstatus1 |= 0b01100;
            }
            Part::Frac8 => {
                // 100
                bufstatus1 |= 0b10000;
            }
            Part::Frac4 => {
                // 101
                bufstatus1 |= 0b10100;
            }
            Part::Frac2 => {
                // 110
                bufstatus1 |= 0b11000;
            }
            Part::Full => {
                // 111
                bufstatus1 |= 0b11100
            }
            // SEC = 1, S6 = 1 for sector protect
            Part::Frac1024 => {
                // 001
                bufstatus1 |= 0b1000100;
            }
            Part::Frac512 => {
                // 010
                bufstatus1 |= 0b1001000;
            }
            Part::Frac256 => {
                // 011
                bufstatus1 |= 0b1001100;
            }
            Part::Frac128 => {
                // 100
                bufstatus1 |= 0b1010000;
            }
        }

        // TB = 1, S5 = 1
        if let Portion::Lower = portion {
            bufstatus1 |= 0b100000
        }
        if complement {
            bufstatus2 |= 0b01000000
        }

        self.cs_low()?;
        self.spi_write(&[0x01, bufstatus1, bufstatus2])?;
        self.cs_high()?;

        Ok(())
    }

    pub fn global_block_lock(&mut self) -> Result<(), Error> {
        self.cs_low()?;
        self.spi_write(&[0x7e])?;
        self.cs_high()?;
        Ok(())
    }
    pub fn global_block_unlock(&mut self) -> Result<(), Error> {
        self.cs_low()?;
        self.spi_write(&[0x98])?;
        self.cs_high()?;
        Ok(())
    }

    /// WARN: It takes 45ms to erase entire block.
    pub fn erase_sector<D: DelayNs>(&mut self, addr: u32, delay: &mut D) -> Result<(), Error> {
        self.write_enable(delay)?;
        if addr
            > match self.model {
                Model::Q128 => 16777216,
                Model::Q64 => 8388608,
                Model::Q32 => 4194304,
                Model::NotSure => 16777216,
            }
        {
            return Ok(());
        }
        let addr = 4096 * (addr / 4096); // make sure erase from starting address
        let mut addrbyte = ((!0 >> 8) & addr).to_be_bytes();
        addrbyte[0] = 0x20;

        // send the erase command
        self.cs_low()?;
        self.spi_write(&addrbyte)?;
        self.cs_high()?;

        delay.delay_ms(45);
        // wait for erasing, it may take long so it need to check the busy bit
        let mut status = [0u8];

        loop {
            self.cs_low()?;
            self.spi_write(&[0x05])?;
            self.spi_read(&mut status)?;
            self.cs_high()?;

            if status[0] & 0b1 == 0 {
                break;
            }
        }

        Ok(())
    }

    /// WARN: It takes 150ms to erase entire block.
    pub fn erase_halfblock<D: DelayNs>(&mut self, addr: u32, delay: &mut D) -> Result<(), Error> {
        self.write_enable(delay)?;
        if addr
            > match self.model {
                Model::Q128 => 16777216,
                Model::Q64 => 8388608,
                Model::Q32 => 4194304,
                Model::NotSure => 16777216,
            }
        {
            return Ok(());
        }
        let addr = 32768 * (addr / 32768); // make sure erase from starting address
        let mut addrbyte = ((!0 >> 8) & addr).to_be_bytes();
        addrbyte[0] = 0x52;

        // send the erase command
        self.cs_low()?;
        self.spi_write(&addrbyte)?;
        self.cs_high()?;

        delay.delay_ms(120);
        // wait for erasing, it may take long so it need to check the busy bit
        let mut status = [0u8];

        loop {
            self.cs_low()?;
            self.spi_write(&[0x05])?;
            self.spi_read(&mut status)?;
            self.cs_high()?;

            if status[0] & 0b1 == 0 {
                break;
            }
        }

        Ok(())
    }
    /// WARN: It takes >150ms to erase entire block.
    pub fn erase_block<D: DelayNs>(&mut self, addr: u32, delay: &mut D) -> Result<(), Error> {
        if addr
            > match self.model {
                Model::Q128 => 16777216,
                Model::Q64 => 8388608,
                Model::Q32 => 4194304,
                Model::NotSure => 16777216,
            }
        {
            return Ok(());
        }
        let addr = 65536 * (addr / 65536); // make sure erase from starting address
        let mut addrbyte = ((!0 >> 8) & addr).to_be_bytes();
        addrbyte[0] = 0xd8;

        self.write_enable(delay)?;
        // send the erase command
        self.cs_low()?;
        self.spi_write(&addrbyte)?;
        self.cs_high()?;

        delay.delay_ms(200);

        // wait for erasing, it may take long so it need to check the busy bit
        let mut status = [0u8];

        loop {
            self.cs_low()?;
            self.spi_write(&[0x05])?;
            self.spi_read(&mut status)?;
            self.cs_high()?;

            if status[0] & 0b1 == 0 {
                break;
            }
        }

        Ok(())
    }
    /// WARN: It takes 40S to erase entire block.
    pub fn erase_chip<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.write_enable(delay)?;
        // send the erase command
        self.cs_low()?;
        self.spi_write(&[0xc7])?;
        self.cs_high()?;

        delay.delay_ms(40000);

        // wait for erasing, it may take long so it need to check the busy bit
        let mut status = [0u8];

        loop {
            self.cs_low()?;
            self.spi_write(&[0x05])?;
            self.spi_read(&mut status)?;
            self.cs_high()?;

            if status[0] & 0b1 == 0 {
                break;
            }
        }

        Ok(())
    }

    pub fn write_program<D: DelayNs>(
        &mut self,
        addr: u32,
        write: &[u8],
        delay: &mut D,
    ) -> Result<(), Error> {
        if write.is_empty() {
            return Ok(());
        }

        // Erase sectors first
        let mut start_sector = addr / 4096;
        let end_sector = (addr + write.len() as u32 - 1) / 4096;

        while start_sector <= end_sector {
            if start_sector % 16 == 0 && start_sector + 16 <= end_sector {
                self.erase_block(start_sector * 4096, delay)?;
                start_sector += 16;
            } else if start_sector % 8 == 0 && start_sector + 8 <= end_sector {
                self.erase_halfblock(start_sector * 4096, delay)?;
                start_sector += 8;
            } else {
                self.erase_sector(start_sector * 4096, delay)?;
                start_sector += 1;
            }
        }

        // Write data in page-sized chunks
        const PAGE_SIZE: usize = 256;
        let mut current_addr = addr;
        let mut remaining_data = write;

        while !remaining_data.is_empty() {
            let page_offset = (current_addr % PAGE_SIZE as u32) as usize;
            let bytes_to_write = core::cmp::min(PAGE_SIZE - page_offset, remaining_data.len());

            let mut addrbyte = ((!0 >> 8) & current_addr).to_be_bytes();
            addrbyte[0] = 0x02;

            self.write_enable(delay)?;
            self.cs_low()?;
            self.spi_write(&addrbyte)?;
            self.spi_write(&remaining_data[..bytes_to_write])?;
            self.cs_high()?;

            delay.delay_ms(5);

            // Wait for write completion
            let mut status = [0u8];
            loop {
                self.cs_low()?;
                self.spi_write(&[0x05])?;
                self.spi_read(&mut status)?;
                self.cs_high()?;

                if status[0] & 0b1 == 0 {
                    break;
                }
            }

            current_addr += bytes_to_write as u32;
            remaining_data = &remaining_data[bytes_to_write..];
        }

        Ok(())
    }
}

pub enum Portion {
    Upper,
    Lower,
}

pub enum Part {
    Zero,
    Frac128,
    Frac256,
    Frac512,
    Frac1024,
    Frac64,
    Frac32,
    Frac16,
    Frac8,
    Frac4,
    Frac2,
    Full,
}

#[derive(Debug, Default, Clone, Copy)]
pub enum SpiSpeed {
    #[default]
    Single = 0x03,
    Fast = 0x0b,
    FastDual = 0x3b,
    FastQuad = 0x6b,
}

#[derive(Debug, Default, Clone)]
pub enum Model {
    #[default]
    NotSure,
    Q128,
    Q64,
    Q32,
}
