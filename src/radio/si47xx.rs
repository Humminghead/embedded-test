use defmt::{bitflags, debug, Format};
use embedded_hal::i2c::I2c;

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum Command {
    PowerUp = 0x01,
    GetRev = 0x10,
    PowerDown = 0x11,
    SetProperty = 0x12,
    GetIntStatus = 0x14,
    FmTuneFreq = 0x20,
    FmSeekStart = 0x21,
    FmTuneStatus = 0x22,
    FmRsqStatus = 0x23,
    FmRdsStatus = 0x24,
    AmTuneFreq = 0x40,
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
    RxVolume = 0x4000,
    RxHardMute = 0x4001,
}

// AN332 (REV 1.0); page 65, ARG1 of POWER_UP
bitflags! {
    pub struct PowerUpArg: u8 {
        const CTSIEN  = 0x80;
        const GPO2OEN = 0x40;
        const PATCH   = 0x20;
        const XOSCEN  = 0x10;
        const FUNC    = 0x0F;
    }
}

// AN332 (REV 1.0); Table 10 - Status Response for the FM/RDS Receiver
bitflags! {
    pub struct ReceiverStatus: u8 {
        const CTS    = 0x80;
        const ERR    = 0x40;
        const RSQINT = 0x08;
        const RDSINT = 0x04;
        const STCINT = 0x01;
    }
}

// AN332 (REV 1.0); page 84 - GPO_IEN
bitflags! {
    pub struct GpoIen: u16 {
        const STC_IEN = 0x0001;
        const RDS_IEN = 0x0004;
        const RSQ_IEN = 0x0008;
        const ERR_IEN = 0x0040;
        const CTS_IEN = 0x0080;
        const STC_REP = 0x0100;
        const RDS_REP = 0x0400;
        const RSQ_REP = 0x0800;
    }
}

// AN332 (REV 1.0); Table 14 - AM/SW/LW receiver status
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
    AmReceive = 1,
    FmTransmit = 2,
    WbReceive = 3,
    AuxIn = 4,
    QueryLibId = 15,
}

pub enum SeekDirection {
    PowerDown = 0,
    Up = 1,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum OptMode {
    RdsOnly = 0b0000_0000,
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

/// The ERR bit (and optional interrupt) is set if an invalid argument is sent.
pub fn is_bus_error(status: u8) -> bool {
    status & ReceiverStatus::ERR.bits() != 0
}

/// The CTS bit (and optional interrupt) is set when it is safe to send the next command
pub fn is_bus_cts(status: u8) -> bool {
    status & ReceiverStatus::CTS.bits() == ReceiverStatus::CTS.bits()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct RevisionResponse {
    pub pn: u8,
    pub fw_major: u8,
    pub fw_minor: u8,
    pub patch_h: u8,
    pub patch_l: u8,
    pub cmp_major: u8,
    pub cmp_minor: u8,
    pub chiprev: u8,
}

const NO_ARGS: &[u8;0] = &[];

impl RevisionResponse {
    fn ascii_digit_to_u8(digit: &u8) -> u8 {
        if digit.is_ascii_digit() {
            digit - b'0'
        } else {
            0
        }
    }

    pub fn from_bytes(data: &[u8]) -> Result<Self, &'static str> {
        if data.len() != 8 {
            return Err("Wrong data length!");
        }

        Ok(RevisionResponse {
            pn: data[0],
            fw_major: Self::ascii_digit_to_u8(&data[1]),
            fw_minor: Self::ascii_digit_to_u8(&data[2]),
            patch_h: data[3],
            patch_l: data[4],
            cmp_major: Self::ascii_digit_to_u8(&data[5]),
            cmp_minor: Self::ascii_digit_to_u8(&data[6]),
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

    /// Writes `cmd` + up to 7 arguments, then reads `N` response bytes.
    ///
    /// AN332 (REV 1.0); page 6: "The system controller may write up to 8 data bytes
    /// in a single 2-wire transaction. The first byte is a command, and the next
    /// seven bytes are arguments."
    async fn send_command<const N: usize>(
        &mut self,
        cmd: u8,
        args: &[u8],
    ) -> Result<[u8; N], ReceiverError<E>> {
        let mut buf: [u8; 8] = [0x00; 8];
        let arg_len = args.len().min(buf.len() - 1); // max 7 arguments

        buf[0] = cmd;
        buf[1..1 + arg_len].copy_from_slice(&args[..arg_len]);

        let mut response = [0u8; N];
        self.bus
            .write_read(self.address, &buf[..1 + arg_len],  response.as_mut_slice())
            .map_err(ReceiverError::I2c)?;

        Ok(response)
    }

    /// Initiates the boot process to move the device from powerdown to powerup mode
    /// 
    /// AN332 (REV 1.0); page 64
    pub async fn power_up(
        &mut self,
        arg1: u8,
        arg2: OptMode,
    ) -> Result<ReceiverStatus, ReceiverError<E>> {
        let resp = self
            .send_command::<1>(Command::PowerUp as u8, [arg1 as u8, arg2 as u8].as_mut_slice())
            .await?;
        Ok(ReceiverStatus::from_bits(resp[0]).unwrap_or(ReceiverStatus::empty()))
    }

    /// Updates bits 6:0 of the status byte.
    /// 
    /// AN332 (REV 1.0); page 70
    pub async fn get_int_status(&mut self) -> Result<ReceiverStatus, ReceiverError<E>> {
        let resp = self
            .send_command::<1>(Command::GetIntStatus as u8, NO_ARGS.as_slice())
            .await?;
        Ok(ReceiverStatus::from_bits(resp[0]).unwrap_or(ReceiverStatus::empty()))
    }

    /// Moves the device from powerup to powerdown mode.
    /// 
    /// AN332 (REV 1.0); page 67
    pub async fn power_down(&mut self) -> Result<ReceiverStatus, ReceiverError<E>> {
        let resp = self
            .send_command::<1>(Command::PowerDown as u8, NO_ARGS.as_slice())
            .await?;
        Ok(ReceiverStatus::from_bits(resp[0]).unwrap_or(ReceiverStatus::empty()))
    }

    /// Returns the part number, chip revision, firmware revision, patch revision and component revision numbers.
    /// 
    /// AN332 (REV 1.0); page 66
    pub async fn get_rev_info(&mut self) -> Result<RevisionResponse, ReceiverError<E>> {
        let data = self.send_command::<9>(Command::GetRev as u8, NO_ARGS.as_slice()).await?;

        if !is_bus_cts(data[0]) {
            return Err(ReceiverError::CtsTimeout);
        }

        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&data[1..]);

        RevisionResponse::from_bytes(&bytes.as_mut_slice()).map_err(|_| ReceiverError::InvalidArg)
    }

    /// Sets a property shown in Table 9, “FM/RDS Receiver Property Summary,” on AN332 page 56.
    /// 
    /// AN332 (REV 1.0); page 68
    pub async fn set_property(
        &mut self,
        property: u16,
        value: u16,
    ) -> Result<ReceiverStatus, ReceiverError<E>> {
        let args: [u8; 5] = [
            0x00,                     // Reserved. Always 0.
            (property >> 8) as u8,    // PROPH
            (property & 0xFF) as u8,  // PROPL
            (value >> 8) as u8,       // PROPDH
            (value & 0xFF) as u8,     // PROPDL
        ];

        let result = self
            .send_command::<1>(Command::SetProperty as u8, args.as_slice())
            .await?;
        Ok(ReceiverStatus::from_bits(result[0]).unwrap_or(ReceiverStatus::empty()))
    }

    /// Sets the FM Receive to tune a frequency between 64 and 108 MHz in 10 kHz units.
    ///     
    /// AN332 (REV 1.0); page 70
    pub async fn set_tune_freq(&mut self, freq: u16) -> Result<ReceiverStatus, ReceiverError<E>> {
        let args: [u8; 4] = [
            0x00,                 // ARG1: FAST=0, FREEZE=0
            (freq >> 8) as u8,    // ARG2: FREQH
            (freq & 0xFF) as u8,  // ARG3: FREQL
            0x00,                 // ARG4: ANTCAP = 0 -> auto
        ];

        let result = self
            .send_command::<1>(Command::FmTuneFreq as u8, args.as_slice())
            .await?;
        Ok(ReceiverStatus::from_bits(result[0]).unwrap_or(ReceiverStatus::empty()))
    }

    /// Begins searching for a valid frequency. Clears any pending STCINT or RSQINT interrupt status.
    ///     
    /// AN332 (REV 1.0); page 72
    pub async fn fm_seek_start(
        &mut self,        
        dir: SeekDirection,
        wrap: bool,
    ) -> Result<ReceiverStatus, ReceiverError<E>> {
        let args: [u8; 1] = [(dir as u8) << 3 | (wrap as u8) << 2];

        let result = self
            .send_command::<1>(Command::FmSeekStart as u8, args.as_slice())
            .await?;
        Ok(ReceiverStatus::from_bits(result[0]).unwrap_or(ReceiverStatus::empty()))
    }

    /// Returns the status of FM_TUNE_FREQ or FM_SEEK_START commands.
    /// 
    /// AN332 (REV 1.0); page 73
    /// Response bytes:
    ///   [0] STATUS, [1] BLTF/AFCRL/VALID, [2] READFREQH, [3] READFREQL,
    ///   [4] RSSI, [5] SNR, [6] MULT, [7] READANTCAP
    pub async fn fm_tune_status(&mut self, intack: bool) -> Result<[u8; 8], ReceiverError<E>> {
        let arg1 = if intack { 0x01 } else { 0x00 };
        self.send_command::<8>(Command::FmTuneStatus as u8, [arg1].as_slice()).await
    }

    /// Returns status information about the received signal quality.
    /// 
    /// AN332 (REV 1.0); page 75
    /// Response bytes:
    ///   [0] STATUS, [1] INT flags, [2] SMUTE/AFCRL/VALID, [3] PILOT/STBLEND,
    ///   [4] RSSI, [5] SNR, [6] MULT, [7] FREQOFF
    pub async fn fm_rsq_status(&mut self, intack: bool) -> Result<[u8; 8], ReceiverError<E>> {
        let arg1 = if intack { 0x01 } else { 0x00 };
        self.send_command::<8>(Command::FmRsqStatus as u8, [arg1].as_slice()).await
    }

    /// Tunes the AM/SW/LW receive to a frequency between 149 and 23 MHz in 1 kHz steps.
    /// 
    /// AN332 (REV 1.0); page 135    
    pub async fn set_am_tune_freq(
        &mut self,
        freq_khz: u16,
    ) -> Result<AmReceiverStatus, ReceiverError<E>> {
        let args: [u8; 5] = [
            0x00,                          // ARG1: FAST = 0
            (freq_khz >> 8) as u8,         // ARG2: FREQH
            (freq_khz & 0xFF) as u8,       // ARG3: FREQL
            0x00,                          // ARG4: auto antenna cap (high)
            0x00,                          // ARG5: auto antenna cap (low)
        ];

        let r = self
            .send_command::<1>(Command::AmTuneFreq as u8, args.as_slice())
            .await?;
        Ok(AmReceiverStatus::from_bits(r[0]).unwrap_or(AmReceiverStatus::empty()))
    }

    /// Returns the status of AM_TUNE_FREQ or AM_SEEK_START commands.
    /// 
    /// AN332 (REV 1.0); page 139    
    pub async fn am_tune_status(&mut self, intack: bool) -> Result<[u8; 8], ReceiverError<E>> {
        let arg1 = if intack { 0x01 } else { 0x00 };
        self.send_command::<8>(Command::AmTuneStatus as u8, [arg1].as_slice()).await
    }
}