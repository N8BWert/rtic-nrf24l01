//!
//! The Registers of the nRF24L01+
//! 

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Register {
    Config = 0x00,
    EnableAutoAcknowledge = 0x01,
    EnableRx = 0x02,
    SetupAddressWidth = 0x03,
    SetupRetransmit = 0x04,
    RfChannel = 0x05,
    RfSetup = 0x06,
    Status = 0x07,
    ObserveTx = 0x08,
    ReceivedPower = 0x09,
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
    pub fn default_value(&self) -> u8 {
        match self {
            Self::Config => 0b0000_1000,
            Self::EnableAutoAcknowledge => 0b0011_1111,
            Self::EnableRx => 0b0000_0011,
            Self::SetupAddressWidth => 0b0000_0011,
            Self::SetupRetransmit => 0b0000_0011,
            Self::RfChannel => 0b0000_0010,
            Self::RfSetup => 0b0000_1110,
            Self::Status => 0b0000_1110,
            Self::ObserveTx => 0x00,
            Self::ReceivedPower => 0x00,
            Self::RxAddressP0 => 0x00,
            Self::RxAddressP1 => 0x00,
            Self::RxAddressP2 => 0xC3,
            Self::RxAddressP3 => 0xC4,
            Self::RxAddressP4 => 0xC5,
            Self::RxAddressP5 => 0xC6,
            Self::TxAddress => 0x00,
            Self::RxPayloadWidthP0 => 0x00,
            Self::RxPayloadWidthP1 => 0x00,
            Self::RxPayloadWidthP2 => 0x00,
            Self::RxPayloadWidthP3 => 0x00,
            Self::RxPayloadWidthP4 => 0x00,
            Self::RxPayloadWidthP5 => 0x00,
            Self::FifoStatus => 0b0001_0001,
            Self::DynamicPayload => 0x00,
            Self::Feature => 0x00,
        }
    }

    pub fn default_rx_0() -> [u8; 5] { [0xE7, 0xE7, 0xE7, 0xE7, 0xE7] }
    
    pub fn default_rx_1() -> [u8; 5] { [0xC2, 0xC2, 0xC2, 0xC2, 0xC2] }

    pub fn default_tx() -> [u8; 5] { [0xE7, 0xE7, 0xE7, 0xE7, 0xE7] }

    pub fn write_registers() -> [Register; 23] {
        [
            Self::Config,
            Self::EnableAutoAcknowledge,
            Self::EnableRx,
            Self::SetupAddressWidth,
            Self::SetupRetransmit,
            Self::RfChannel,
            Self::RfSetup,
            Self::Status,
            Self::RxAddressP0,
            Self::RxAddressP1,
            Self::RxAddressP2,
            Self::RxAddressP3,
            Self::RxAddressP4,
            Self::RxAddressP5,
            Self::TxAddress,
            Self::RxPayloadWidthP0,
            Self::RxPayloadWidthP1,
            Self::RxPayloadWidthP2,
            Self::RxPayloadWidthP3,
            Self::RxPayloadWidthP4,
            Self::RxPayloadWidthP5,
            Self::Feature,
            Self::DynamicPayload,
        ]
    }

    pub fn payload_length_register(pipe: u8) -> Register {
        match pipe {
            0 => Self::RxPayloadWidthP0,
            1 => Self::RxPayloadWidthP1,
            2 => Self::RxPayloadWidthP2,
            3 => Self::RxPayloadWidthP3,
            4 => Self::RxPayloadWidthP4,
            5 => Self::RxPayloadWidthP5,
            _ => Self::RxPayloadWidthP0,
        }
    }

    pub fn address_register(pipe: u8) -> Register {
        match pipe {
            0 => Self::RxAddressP0,
            1 => Self::RxAddressP1,
            2 => Self::RxAddressP2,
            3 => Self::RxAddressP3,
            4 => Self::RxAddressP4,
            5 => Self::RxAddressP5,
            _ => Self::RxAddressP0,
        }
    }
}