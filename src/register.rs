//!
//! Register Mapping for Ease of Use of the nRF24L01.
//! 

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