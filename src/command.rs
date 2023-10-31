//! Commands that can be sent to the NRF24L01 Module.
//! 
//! Never use these commands on their own and instead use the library functions

/// The various SPI commands that can be issued to the NRF24L01 module
pub enum Command {
    ReadRegister(u8),
    WriteRegister(u8),
    ReadRxPayload,
    ReadTxPayload,
    FlushTx,
    FlushRx,
    ReuseTxPayload,
    ReadRxPayloadWidth,
    WriteAcknowledgePayload(u8),
    DisableAutoackTx,
    Nop,
}

impl Command {
    // Parse the opcode of command from the command itself
    pub fn opcode(&self) -> u8 {
        match self {
            Self::ReadRegister(register) => 0b0000_0000 | register,
            Self::WriteRegister(register) => 0b0010_0000 | register,
            Self::ReadRxPayload => 0b0110_0001,
            Self::ReadTxPayload => 0b1010_0000,
            Self::FlushTx => 0b1110_0001,
            Self::FlushRx => 0b1110_0010,
            Self::ReuseTxPayload => 0b1110_0011,
            Self::ReadRxPayloadWidth => 0b0110_0000,
            Self::WriteAcknowledgePayload(payload) => 0b1010_1000 | payload,
            Self::DisableAutoackTx => 0b1011_0000,
            Self::Nop => 0b1111_1111,
        }
    }
}