//use core::error;
//use core::fmt::Error;

use defmt::{bitflags, Format};
use defmt::{debug, error, info};
//use embassy_sync::channel::Receiver;
use embassy_time::Timer;
use embedded_hal::i2c::I2c;

static TIMEOUT_CTS_WAIT: u64 = 10; //millis

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum Command {
    PowerUp = 0x01,
    PowerDown = 0x11,
    GetRev = 0x10,
    GetIntStatus = 0x14,
    SetProperty = 0x12,
    FmTuneFreq = 0x20,
    AmTuneFreq = 0x40, // AM/SW/LW
    AmSeekStart = 0x41,
    AmTuneStatus = 0x42,
    AmRsqStatus = 0x43,
}

#[repr(u16)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReceiverProperties {
    GpoIen = 0x0001,
    DigitalOutputFormat = 0x0102,
    DigitalOutputSampleRate = 0x0104,
    RefclkFreq = 0x0201,
    RefclkPrescale = 0x0202,
    FmDeemphasis = 0x1100,
    FmChannelFilter = 0x1102,
    FmBlendStereoThreshold = 0x1105,
    FmBlendMonoThreshold = 0x1106,
    FmAntennaInput = 0x1107,
    FmMaxTuneError = 0x1108,
    FmRsqIntSource = 0x1200,
    FmRsqSnrHiThreshold = 0x1201,
    FmRsqSnrLoThreshold = 0x1202,
    FmRsqRssiHiThreshold = 0x1203,
    FmRsqRssiLoThreshold = 0x1204,
    FmRsqMultipathHiThreshold = 0x1205,
    FmRsqMultipathLoThreshold = 0x1206,
    FmRsqBlendThreshold = 0x1207,
    FmSoftMuteRate = 0x1300,
    FmSoftMuteSlope = 0x1301,
    FmSoftMuteMaxAttenuation = 0x1302,
    FmSoftMuteSnrThreshold = 0x1303,
    FmSoftMuteReleaseRate = 0x1304,
    FmSoftMuteAttackRate = 0x1305,
    FmSeekBandBottom = 0x1400,
    FmSeekBandTop = 0x1401,
    FmSeekFreqSpacing = 0x1402,
    FmSeekTuneSnrThreshold = 0x1403,
    FmSeekTuneRssiThreshold = 0x1404,
    RdsIntSource = 0x1500,
    RdsIntFifoCount = 0x1501,
    RdsConfig = 0x1502,
    FmRdsConfidence = 0x1503,
    FmAgcAttackRate = 0x1700,
    FmAgcReleaseRate = 0x1701,
    FmBlendRssiStereoThreshold = 0x1800,
    FmBlendRssiMonoThreshold = 0x1801,
    FmBlendRssiAttackRate = 0x1802,
    FmBlendRssiReleaseRate = 0x1803,
    FmBlendSnrStereoThreshold = 0x1804,
    FmBlendSnrMonoThreshold = 0x1805,
    FmBlendSnrAttackRate = 0x1806,
    FmBlendSnrReleaseRate = 0x1807,
    FmBlendMultipathStereoThreshold = 0x1808,
    FmBlendMultipathMonoThreshold = 0x1809,
    FmBlendMultipathAttackRate = 0x180A,
    FmBlendMultipathReleaseRate = 0x180B,
    FmBlendMaxStereoSeparation = 0x180C,
    FmNbDetectThreshold = 0x1900,
    FmNbInterval = 0x1901,
    FmNbRate = 0x1902,
    FmNbIirFilter = 0x1903,
    FmNbDelay = 0x1904,
    FmHicutSnrHighThreshold = 0x1A00,
    FmHicutSnrLowThreshold = 0x1A01,
    FmHicutAttackRate = 0x1A02,
    FmHicutReleaseRate = 0x1A03,
    FmHicutMultipathTriggerThreshold = 0x1A04,
    FmHicutMultipathEndThreshold = 0x1A05,
    FmHicutCutoffFrequency = 0x1A06,
    RxVolume = 0x4000,
    RxHardMute = 0x4001,
    AmDeemphasis = 0x3100,
    AmChannelFilter = 0x3102,
    AmAutomaticVolumeControlMaxGain = 0x3103,
    AmModeAfcSwPullInRange = 0x3104,
    AmModeAfcSwLockInRange = 0x3105,
    AmRsqInterrupts = 0x3200,
    AmRsqSnrHighThreshold = 0x3201,
    AmRsqSnrLowThreshold = 0x3202,
    AmRsqRssiHighThreshold = 0x3203,
    AmRsqRssiLowThreshold = 0x3204,
    AmSoftMuteRate = 0x3300,
    AmSoftMuteSlope = 0x3301,
    AmSoftMuteMaxAttenuation = 0x3302,
    AmSoftMuteSnrThreshold = 0x3303,
    AmSoftMuteReleaseRate = 0x3304,
    AmSoftMuteAttackRate = 0x3305,
    AmSeekBandBottom = 0x3400,
    AmSeekBandTop = 0x3401,
    AmSeekFreqSpacing = 0x3402,
    AmSeekSnrThreshold = 0x3403,
    AmSeekRssiThreshold = 0x3404,
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

bitflags! {
    pub struct GpoIen: u16 {
        const STC_IEN   = 0x0001;
        const RDS_IEN   = 0x0004;
        const RSQ_IEN   = 0x0008;
        const ERR_IEN   = 0x0040;
        const CTS_IEN   = 0x0080;
        const STC_REP   = 0x0100;
        const RDS_REP   = 0x0400;
        const RSQ_REP   = 0x0800;
    }
}

// AN332 Table 14 - AM/SW/LW receiver status
bitflags! {
    pub struct AmReceiverStatus: u8 {
        const CTS    = 0x80;
        const ERR    = 0x40;
        const RSQINT = 0x08;
        const STCINT = 0x01;
    }
}

impl Default for GpoIen {
    fn default() -> Self {
        GpoIen::empty()
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum ReceiverError<E> {
    I2c(E),
    InvalidArg,
    CtsTimeout,
}

pub fn is_bus_error(status: u8) -> bool {
    status & ReceiverStatus::ERR.bits() != 0
}

pub fn is_bus_cts(status: u8) -> bool {
    status & ReceiverStatus::CTS.bits() == ReceiverStatus::CTS.bits
}

pub async fn wait_cts(status: &ReceiverStatus) -> bool {
    Timer::after_micros(TIMEOUT_CTS_WAIT).await;

    is_bus_cts(status.bits() as u8)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
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
    fn convert_chip_hex_to_digit(digit: &u8) -> u8 {
        if digit.is_ascii_digit() {
            return digit - 0x30;
        }
        0
    }
    pub fn from_bytes(data: &[u8]) -> Result<Self, &'static str> {
        if data.len() != 8 {
            return Err("Wrong data length!");
        }

        Ok(RevisionResponse {
            pn: data[0],
            fw_major: Self::convert_chip_hex_to_digit(&data[1]),
            fw_minor: Self::convert_chip_hex_to_digit(&data[2]),
            patch_h: data[3],
            patch_l: data[4],
            cmp_major: Self::convert_chip_hex_to_digit(&data[5]),
            cmp_minor: Self::convert_chip_hex_to_digit(&data[6]),
            chiprev: data[7],
        })
    }
}

pub struct FmReceiver<I2C> {
    bus: I2C,
    address: u8,
}

impl<I2C, E> FmReceiver<I2C>
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
        let mut buf: [u8; 8] = [0x00; 8];
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
        self.bus
            .write_read(self.address, &buf[..1 + arg_len], &mut responce)
            .map_err(ReceiverError::I2c)?;

        return Ok(responce);
    }

    pub async fn power_up(
        &mut self,
        arg1: u8,
        arg2: OptMode,
    ) -> Result<ReceiverStatus, ReceiverError<E>> {
        let resp = self
            .send_command::<1>(Command::PowerUp as u8, &[arg1, arg2 as u8])
            .await?;

        Ok(ReceiverStatus::from_bits(resp[0]).unwrap_or(ReceiverStatus::empty()))
    }

    pub async fn get_int_status(&mut self) -> Result<ReceiverStatus, ReceiverError<E>> {
        let resp = self
            .send_command::<1>(Command::GetIntStatus as u8, &[])
            .await?;

        Ok(ReceiverStatus::from_bits(resp[0]).unwrap_or(ReceiverStatus::empty()))
    }

    pub async fn power_down(&mut self) -> Result<ReceiverStatus, ReceiverError<E>> {
        let resp = self
            .send_command::<1>(Command::PowerDown as u8, &[])
            .await?;

        Ok(ReceiverStatus::from_bits(resp[0]).unwrap_or(ReceiverStatus::empty()))
    }

    pub async fn get_rev_info(&mut self) -> Result<RevisionResponse, ReceiverError<E>> {
        let data = self.send_command::<9>(Command::GetRev as u8, &[]).await?;

        if !is_bus_cts(data[0]) {
            return Err(ReceiverError::CtsTimeout);
        }

        let mut bytes: [u8; 8] = [0x00; 8];
        bytes.copy_from_slice(&data[1..]);

        let result = RevisionResponse::from_bytes(&bytes);

        Ok(result.unwrap())
    }

    pub async fn set_property(
        &mut self,
        property: u16,
        value: u16,
    ) -> Result<ReceiverStatus, ReceiverError<E>> {
        let mut buf: [u8; 5] = [0x00; 5];
        buf.fill(0x00);
        buf[0] = 0x00; // Reserved. Always write to 0.
        buf[2] = (property & 0xFF) as u8; // property low byte
        buf[1] = (property >> 8) as u8; // property high byte
        buf[4] = (value & 0xFF) as u8; // value low byte
        buf[3] = (value >> 8) as u8; // value high byte

        let result = self
            .send_command::<1>(Command::SetProperty as u8, &buf)
            .await?;

        Ok(ReceiverStatus::from_bits(result[0]).unwrap_or(ReceiverStatus::empty()))
    }

    pub async fn set_tune_freq(&mut self, freq: u16) -> Result<ReceiverStatus, ReceiverError<E>> {
        let mut buf: [u8; 4] = [0x00; 4];
        buf.fill(0x00);
        buf[0] = 0x00;
        buf[1] = (freq >> 8) as u8; // property high byte
        buf[2] = (freq & 0xFF) as u8; // property low byte
        buf[3] = 0;

        debug!(
            "Set FM freq to: 0x{:X}{:X} ({})",
            (freq >> 8) as u8,
            (freq & 0xFF) as u8,
            freq
        );

        let result = self
            .send_command::<1>(Command::FmTuneFreq as u8, &buf)
            .await?;

        Ok(ReceiverStatus::from_bits(result[0]).unwrap_or(ReceiverStatus::empty()))
    }

    pub async fn set_am_tune_freq(
        &mut self,
        freq_khz: u16,
    ) -> Result<AmReceiverStatus, ReceiverError<E>> {
        // CMD 0x40, ARG1 = FAST bit (0x01) or 0
        // ARG2 = FREQH, ARG3 = FREQL, ARG4 = ANTCAPH, ARG5 = ANTCAPL
        let args: [u8; 5] = [
            0x00,                    // ARG1: FAST = 0
            (freq_khz >> 8) as u8,   // ARG2: FREQH
            (freq_khz & 0xFF) as u8, // ARG3: FREQL
            0x00,                    // ARG4: auto antenna cap (high)
            0x00,                    // ARG5: auto antenna cap (low)
        ];

        let r = self
            .send_command::<1>(Command::AmTuneFreq as u8, &args)
            .await?;

        Ok(AmReceiverStatus::from_bits(r[0]).unwrap_or(AmReceiverStatus::empty()))
    }

    pub async fn am_tune_status(&mut self, intack: bool) -> Result<[u8; 8], ReceiverError<E>> {
        let arg1 = if intack { 0x01 } else { 0x00 };
        let r = self
            .send_command::<8>(Command::AmTuneStatus as u8, &[arg1])
            .await?;
        Ok(r)
    }
}
