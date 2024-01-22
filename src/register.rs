//!
//! Register Mapping for Ease of Use of the nRF24L01.
//! 

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Register {
    Config = 0x00,
    EnableAutoAcknowledge = 0x01,
    EnabledRxAddresses = 0x02,
    SetupAddressWidths = 0x03,
    SetupAutoRetransmit = 0x04,
    RfChannel = 0x05,
    RfSetup = 0x06,
    Status = 0x07,
    ObserveTx = 0x08,
    ReceivedPowerDetector = 0x09,
    RxAddressP0 = 0x0A,
    RxAddressP1 = 0x0B,
    RxAddressP2 = 0x0C,
    RxAddressP3 = 0x0D,
    RxAddressP4 = 0x0E,
    RxAddressP5 = 0x0F,
    TxAddress = 0x10,
    RxPayloadWidthP0 = 0x11,
    RxPayloadWidthP1 = 0x12,
    RxPayloadWidthP2 = 0x13,
    RxPayloadWidthP3 = 0x14,
    RxPayloadWidthP4 = 0x15,
    RxPayloadWidthP5 = 0x16,
    FifoStatus = 0x17,
    DynamicPayload = 0x1C,
    Feature = 0x1D,
}

impl Register {
    pub fn get_width_registers() -> [Self; 6] {
        [Self::RxPayloadWidthP0, Self::RxPayloadWidthP1,
         Self::RxPayloadWidthP2, Self::RxPayloadWidthP3,
         Self::RxPayloadWidthP4, Self::RxPayloadWidthP5]
    }

    pub fn get_writable_byte_registers() -> [Self; 18] {
        [
            Self::Config,
            Self::EnableAutoAcknowledge,
            Self::EnabledRxAddresses,
            Self::SetupAddressWidths,
            Self::SetupAutoRetransmit,
            Self::RfChannel,
            Self::RfSetup,
            Self::Status,
            Self::RxAddressP2,
            Self::RxAddressP3,
            Self::RxAddressP4,
            Self::RxAddressP5,
            Self::RxPayloadWidthP0,
            Self::RxPayloadWidthP1,
            Self::RxPayloadWidthP2,
            Self::RxPayloadWidthP3,
            Self::RxPayloadWidthP4,
            Self::RxPayloadWidthP5,
        ]
    }

    pub fn get_payload_length(pipe: u8) -> Self {
        match pipe {
            0 => Self::RxPayloadWidthP0,
            1 => Self::RxPayloadWidthP1,
            2 => Self::RxPayloadWidthP2,
            3 => Self::RxPayloadWidthP3,
            4 => Self::RxPayloadWidthP4,
            5 => Self::RxPayloadWidthP5,
            _ => Self::RxPayloadWidthP5
        }
    }

    pub fn get_rx_address(pipe: u8) -> Self {
        match pipe {
            0 => Self::RxAddressP0,
            1 => Self::RxAddressP1,
            2 => Self::RxAddressP2,
            3 => Self::RxAddressP3,
            4 => Self::RxAddressP4,
            5 => Self::RxAddressP5,
            _ => Self::RxAddressP0
        }
    }

    pub fn get_default(self) -> u8 {
        match self {
            Self::Config => 0b0000_1000,
            Self::EnableAutoAcknowledge => 0b0011_1111,
            Self::EnabledRxAddresses => 0b0000_0011,
            Self::SetupAddressWidths => 0b0000_0011,
            Self::SetupAutoRetransmit => 0b0000_0011,
            Self::RfChannel => 0b0000_0010,
            Self::RfSetup => 0b0000_1110,
            Self::Status => 0b0000_1110,
            Self::ObserveTx => 0,
            Self::ReceivedPowerDetector => 0,
            Self::RxAddressP2 => 0xC3,
            Self::RxAddressP3 => 0xC4,
            Self::RxAddressP4 => 0xC5,
            Self::RxAddressP5 => 0xC6,
            Self::RxPayloadWidthP0 => 0,
            Self::RxPayloadWidthP1 => 0,
            Self::RxPayloadWidthP2 => 0,
            Self::RxPayloadWidthP3 => 0,
            Self::RxPayloadWidthP4 => 0,
            Self::RxPayloadWidthP5 => 0,
            Self::FifoStatus => 0b0001_0001,
            Self::DynamicPayload => 0,
            Self::Feature => 0,
            _ => 0,
        }
    }
}

pub trait ContainsStatus<Status> {
    fn contains_status(&self, status: Status) -> bool;
}

pub trait StatusValue<Status> {
    fn get_status_value(&self, status: Status) -> u8;
}

pub enum StatusRegister {
    RxDataReady = 0b0100_0000,
    TxDataSent = 0b0010_0000,
    MaxRetriesMet = 0b0001_0000,
    RxPipeNum = 0b0000_1110,
    TxFull = 0b0000_0001,
}

impl ContainsStatus<StatusRegister> for u8 {
    fn contains_status(&self, status: StatusRegister) -> bool {
        *self & status as u8 != 0
    }
}

pub enum ConfigRegister {
    MaskRxInterrupt = 0b0100_0000,
    MaskTxInterrupt = 0b0010_0000,
    MaskRetriesInterrupt = 0b0001_0000,
    EnableCRC = 0b0000_1000,
    CRCEncoding = 0b0000_0100,
    PowerUp = 0b0000_0010,
    RxTxControl = 0b0000_0001,
}

impl ContainsStatus<ConfigRegister> for u8 {
    fn contains_status(&self, status: ConfigRegister) -> bool {
        *self & status as u8 != 0
    }
}

pub enum AutoAcknowledgeRegister {
    P5 = 0b0010_0000,
    P4 = 0b0001_0000,
    P3 = 0b0000_1000,
    P2 = 0b0000_0100,
    P1 = 0b0000_0010,
    P0 = 0b0000_0001,
}

impl ContainsStatus<AutoAcknowledgeRegister> for u8 {
    fn contains_status(&self, status: AutoAcknowledgeRegister) -> bool {
        *self & status as u8 != 0
    }
}

pub enum EnableDataPipeRegister {
    P5 = 0b0010_0000,
    P4 = 0b0001_0000,
    P3 = 0b0000_1000,
    P2 = 0b0000_0100,
    P1 = 0b0000_0010,
    P0 = 0b0000_0001,
}

impl ContainsStatus<EnableDataPipeRegister> for u8 {
    fn contains_status(&self, status: EnableDataPipeRegister) -> bool {
        *self & status as u8 != 0
    }
}

pub enum AddressWidthRegister {
    AddressWidth = 0b0000_0011,
}

impl StatusValue<AddressWidthRegister> for u8 {
    fn get_status_value(&self, status: AddressWidthRegister) -> u8 {
        *self & status as u8
    }
}

pub enum RetransmissionRegister {
    RetransmitDelay = 0b1111_0000,
    RetransmitCount = 0b0000_1111,
}

impl StatusValue<RetransmissionRegister> for u8 {
    fn get_status_value(&self, status: RetransmissionRegister) -> u8 {
        *self & status as u8
    }
}

pub enum ChannelRegister {
    Channel = 0b0111_1111,
}

impl StatusValue<ChannelRegister> for u8 {
    fn get_status_value(&self, status: ChannelRegister) -> u8 {
        *self & status as u8
    }
}

pub enum SetupRegister {
    ContinuousCarrier = 0b1000_0000,
    RfLow = 0b0010_0000,
    PLLLock = 0b0001_0000,
    RFHigh = 0b0000_1000,
    Power = 0b0000_0110,
}

impl ContainsStatus<SetupRegister> for u8 {
    fn contains_status(&self, status: SetupRegister) -> bool {
        *self & status as u8 != 0
    }
}

pub enum TransmitObserveRegister {
    LostPackets = 0b1111_0000,
    RetransmittedPackets = 0b0000_1111,
}

impl StatusValue<TransmitObserveRegister> for u8 {
    fn get_status_value(&self, status: TransmitObserveRegister) -> u8 {
        *self & status as u8
    }
}

pub enum ReceivedPowerRegister {
    ReceivedPowerDetector = 0b0000_0001,
}

impl ContainsStatus<ReceivedPowerRegister> for u8 {
    fn contains_status(&self, status: ReceivedPowerRegister) -> bool {
        *self & status as u8 != 0
    }
}

pub enum FifoStatusRegister {
    TxReuse = 0b0100_0000,
    TxFull = 0b0010_0000,
    TxEmpty = 0b0001_0000,
    RxFull = 0b0000_0010,
    RxEmpty = 0b0000_0001,
}

impl ContainsStatus<FifoStatusRegister> for u8 {
    fn contains_status(&self, status: FifoStatusRegister) -> bool {
        *self & status as u8 != 0
    }
}

pub enum DynamicPayloadRegister {
    P5 = 0b0010_0000,
    P4 = 0b0001_0000,
    P3 = 0b0000_1000,
    P2 = 0b0000_0100,
    P1 = 0b0000_0010,
    P0 = 0b0000_0001,
}

impl ContainsStatus<DynamicPayloadRegister> for u8 {
    fn contains_status(&self, status: DynamicPayloadRegister) -> bool {
        *self & status as u8 != 0
    }
}

pub enum FeatureRegister {
    DynamicPayload = 0b0000_0100,
    AcknowledgePayload = 0b0000_0010,
    DynamicAcknowledge = 0b0000_0001,
}

impl ContainsStatus<FeatureRegister> for u8 {
    fn contains_status(&self, status: FeatureRegister) -> bool {
        *self & status as u8 != 0
    }
}