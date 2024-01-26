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

    pub fn all() -> [Register; 26] {
        [
            Self::Config,
            Self::EnableAutoAcknowledge,
            Self::EnableRx,
            Self::SetupAddressWidth,
            Self::SetupRetransmit,
            Self::RfChannel,
            Self::RfSetup,
            Self::Status,
            Self::ObserveTx,
            Self::ReceivedPower,
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
            Self::FifoStatus,
            Self::DynamicPayload,
            Self::Feature,
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

/// For debugging this is an implementation of the various registers with an accompanied value
/// to use the debug formatter.
pub struct RegisterMap {
    pub config: u8,
    pub en_aa: u8,
    pub en_rxaddr: u8,
    pub setup_aw: u8,
    pub setup_retr: u8,
    pub rf_ch: u8,
    pub rf_setup: u8,
    pub status: u8,
    pub observe_tx: u8,
    pub rpd: u8,
    pub rx_addr_p0: [u8; 5],
    pub rx_addr_p1: [u8; 5],
    pub rx_addr_p2: u8,
    pub rx_addr_p3: u8,
    pub rx_addr_p4: u8,
    pub rx_addr_p5: u8,
    pub tx_addr: [u8; 5],
    pub rx_pw_p0: u8,
    pub rx_pw_p1: u8,
    pub rx_pw_p2: u8,
    pub rx_pw_p3: u8,
    pub rx_pw_p4: u8,
    pub rx_pw_p5: u8,
    pub fifo_status: u8,
    pub dynpd: u8,
    pub feature: u8,
}

impl RegisterMap {
    pub fn add_register_value(&mut self, register: Register, value: u8) {
        match register {
            Register::Config => self.config = value,
            Register::EnableAutoAcknowledge => self.en_aa = value,
            Register::EnableRx => self.en_rxaddr = value,
            Register::SetupAddressWidth => self.setup_aw = value,
            Register::SetupRetransmit => self.setup_retr = value,
            Register::RfChannel => self.rf_ch = value,
            Register::RfSetup => self.rf_setup = value,
            Register::Status => self.status = value,
            Register::ObserveTx => self.observe_tx = value,
            Register::ReceivedPower => self.rpd = value,
            Register::RxAddressP2 => self.rx_addr_p2 = value,
            Register::RxAddressP3 => self.rx_addr_p3 = value,
            Register::RxAddressP4 => self.rx_addr_p4 = value,
            Register::RxAddressP5 => self.rx_addr_p5 = value,
            Register::RxPayloadWidthP0 => self.rx_pw_p0 = value,
            Register::RxPayloadWidthP1 => self.rx_pw_p1 = value,
            Register::RxPayloadWidthP2 => self.rx_pw_p2 = value,
            Register::RxPayloadWidthP3 => self.rx_pw_p3 = value,
            Register::RxPayloadWidthP4 => self.rx_pw_p4 = value,
            Register::RxPayloadWidthP5 => self.rx_pw_p5 = value,
            Register::FifoStatus => self.fifo_status = value,
            Register::DynamicPayload => self.dynpd = value,
            Register::Feature => self.feature = value,
            _ => (),
        }
    }

    pub fn add_array_value(&mut self, register: Register, value: [u8; 5]) {
        match register {
            Register::RxAddressP0 => self.rx_addr_p0 = value,
            Register::RxAddressP1 => self.rx_addr_p1 = value,
            Register::TxAddress => self.tx_addr = value,
            _ => (),
        }
    }
}

impl Default for RegisterMap {
    fn default() -> Self {
        Self {
            config: 0,
            en_aa: 0,
            en_rxaddr: 0,
            setup_aw: 0,
            setup_retr: 0,
            rf_ch: 0,
            rf_setup: 0,
            status: 0,
            observe_tx: 0,
            rpd: 0,
            rx_addr_p0: [0u8; 5],
            rx_addr_p1: [0u8; 5],
            rx_addr_p2: 0,
            rx_addr_p3: 0,
            rx_addr_p4: 0,
            rx_addr_p5: 0,
            tx_addr: [0u8; 5],
            rx_pw_p0: 0,
            rx_pw_p1: 0,
            rx_pw_p2: 0,
            rx_pw_p3: 0,
            rx_pw_p4: 0,
            rx_pw_p5: 0,
            fifo_status: 0,
            dynpd: 0,
            feature: 0
        }
    }
}

impl core::fmt::Debug for RegisterMap {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("RegisterMap")
            .field("\nCONFIG", &format_args!("{:#02X}", self.config))
            .field("\nEN_AA", &format_args!("{:#02X}", self.en_aa))
            .field("\nEN_RXADDR", &format_args!("{:#02X}", self.en_rxaddr))
            .field("\nSETUP_AW", &format_args!("{:#02X}", self.setup_aw))
            .field("\nSETUP_RETR", &format_args!("{:#02X}", self.setup_retr))
            .field("\nRF_CH", &format_args!("{:#02X}", self.rf_ch))
            .field("\nRF_SETUP", &format_args!("{:#02X}", self.rf_setup))
            .field("\nSTATUS", &format_args!("{:#02X}", self.status))
            .field("\nOBSERVE_TX", &format_args!("{:#02X}", self.observe_tx))
            .field("\nRPD", &format_args!("{:#02X}", self.rpd))
            .field("\nRX_ADDR_P0", &format_args!("{:#02X}{:02X}{:02X}{:02X}{:02X}",
                                                            self.rx_addr_p0[0], self.rx_addr_p0[1], self.rx_addr_p0[2], self.rx_addr_p0[3], self.rx_addr_p0[4]))
            .field("\nRX_ADDR_P1", &format_args!("{:#02X}{:02X}{:02X}{:02X}{:02X}",
                                                            self.rx_addr_p1[0], self.rx_addr_p1[1], self.rx_addr_p1[2], self.rx_addr_p1[3], self.rx_addr_p1[4]))
            .field("\nRX_ADDR_P2-5", &format_args!("{:#02X} {:#02X} {:#02X} {:#02X}",
                                                            self.rx_addr_p2, self.rx_addr_p3, self.rx_addr_p4, self.rx_addr_p5))
            .field("\nTX_ADDR", &format_args!("{:#02X}{:02X}{:02X}{:02X}{:02X}",
                                                            self.tx_addr[0], self.tx_addr[1], self.tx_addr[2], self.tx_addr[3], self.tx_addr[4]))
            .field("\nRX_PW_P0-P5", &format_args!("{:#02X} {:#02X} {:#02X} {:#02X} {:#02X} {:#02X}",
                                                            self.rx_pw_p0, self.rx_pw_p1, self.rx_pw_p2, self.rx_pw_p3, self.rx_pw_p4, self.rx_pw_p5))
            .field("\nFIFO_STATUS", &format_args!("{:#02X}", self.fifo_status))
            .field("\nDYNPD", &format_args!("{:#02X}", self.dynpd))
            .field("\nFEATURE", &format_args!("{:#02X}", self.feature))
            .finish()
    }
}