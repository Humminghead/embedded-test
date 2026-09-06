use defmt::bitflags;
use embedded_hal::i2c::I2c;

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum Command {
    PowerUp = 0x01,
    PowerDown = 0x11,
    GetRev = 0x10,
}

bitflags! {
  pub struct PowerUpArg:u8 {
      const CTSIEN = 0x80;
      const GPO2OEN = 0x40;
      const PATCH = 0x20;
      const XOSCEN = 0x10;
      const FUNC = 0x0F;
  }
}

// AN332
// Table 10. Status Response for the FM/RDS Receiver
bitflags! {
  pub struct ReceiverStatus:u8 {
      const CTS = 0x80;
      const ERR = 0x40;
      const RES0 = 0x20;
      const RES1 = 0x10;
      const RSQINT = 0x08;
      const RDSINT = 0x04;
      const RES2 = 0x02;
      const STCINT = 0x01;
  }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum Function {
    FmReceive = 0,
    AmReceive,
    FmTrasmit,
    WbReceive,
    AuxIn,
    QueryLibId = 15,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum OptMode {
    RdsOnly = 0,
    AnalogAudio = 0b0000_0101,             // Analog audio
    DigitalAudio = 0b0000_1011,            // Digital audio output (DCLK, LOUT/DFS, ROUT/DIO)
    DigitalAudioFmRx2 = 0b1011_0000,       // Digital audio output (DCLK, DFS, DIO)
    AnalogDigitalAudioFmRx2 = 0b1011_0101, // Analog and digital audio outputs (LOUT/ROUT and DCLK, DFS,DIO)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReceiverError<E> {
    I2c(E),
    InvalidArg,
    CtsTimeout,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RevisionResponse {
    /// Part Number (PN[7:0])
    pub pn: u8,
    /// Firmware major version
    pub fw_major: u8,
    /// Firmware minor version
    pub fw_minor: u8,
    /// Patch high byte
    pub patch_h: u8,
    /// Patch low byte
    pub patch_l: u8,
    /// Component major version
    pub cmp_major: u8,
    /// Component minor version
    pub cmp_minor: u8,
    /// Chip revision
    pub chiprev: u8,
    // (Optional: CID for Si4705 – if needed, add field)
}

impl RevisionResponse {
    pub fn from_bytes(data: &[u8]) -> Result<Self, &'static str> {
        if data.len() != 8 {
            return Err("Wrong data length!");
        }

        Ok(RevisionResponse {
            pn: data[0],
            fw_major: data[1],
            fw_minor: data[2],
            patch_h: data[3],
            patch_l: data[4],
            cmp_major: data[5],
            cmp_minor: data[6],
            chiprev: data[7],
        })
    }
}

pub struct Receiver<I2C> {
    bus: I2C,
    address: u8,
}

impl<I2C, E> Receiver<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(bus: I2C, address: u8) -> Self {
        Self { bus, address }
    }

    async fn send_command<const N: usize>(
        &mut self,
        cmd: u8,
        args: &[u8],
    ) -> Result<[u8; N], ReceiverError<E>> {
        // create buffer and fill it with zeroies
        let mut buf: [u8; 8] = [cmd; 8];
        buf.fill(0);

        // check buffer's out of boundary
        let arg_len = args.len().min(buf.len());

        // copies the cmd in the buffer
        buf[0] = cmd;

        // copies the arguments into the buffer starting at index 1 (after the command byte).
        buf[1..1 + arg_len].copy_from_slice(&args[..arg_len]);

        // create responce buffer
        let mut responce = [0; N];

        // send command + arguments
        match self
            .bus
            .write_read(self.address, &buf[..1 + arg_len], &mut responce)
        {
            Ok(_) => {
                return Ok(responce);
            }

            Err(e) => {
                return Err(ReceiverError::I2c(e));
            }
        }
    }

    fn check_bus_status_byte(&mut self, status: u8) -> Result<(), ReceiverError<E>> {
        let cts = (status & ReceiverStatus::CTS.bits()) == ReceiverStatus::CTS.bits();
        let err = (status & ReceiverStatus::ERR.bits()) == ReceiverStatus::ERR.bits();

        if err {
            return Err(ReceiverError::InvalidArg);
        }

        if !cts {
            return Err(ReceiverError::CtsTimeout);
        }

        Ok(())
    }

    pub async fn power_up(&mut self, mode: OptMode) -> Result<(), ReceiverError<E>> {
        let resp = self
            .send_command::<1>(
                Command::PowerUp as u8,
                &[PowerUpArg::empty().bits(), mode as u8],
            )
            .await?;

        self.check_bus_status_byte(resp[0])?;

        Ok(())
    }

    pub async fn power_down(&mut self) -> Result<(), ReceiverError<E>> {
        let resp = self
            .send_command::<1>(Command::PowerDown as u8, &[])
            .await?;

        self.check_bus_status_byte(resp[0])?;

        Ok(())
    }

    pub async fn get_rev_info(&mut self) -> Result<RevisionResponse, ReceiverError<E>> {
        let data = self.send_command::<9>(Command::GetRev as u8, &[]).await?;
        self.check_bus_status_byte(data[0])?;

        let mut bytes: [u8; 8] = [0x00; 8];
        bytes.copy_from_slice(&data[1..]);

        let result = RevisionResponse::from_bytes(&bytes);

        if result.is_err() {
            return Err(ReceiverError::InvalidArg);
        }

        Ok(result.unwrap())
    }
}
