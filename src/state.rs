//! State Representation of the NRF24L01 Module

/// The State of the NRF24L01 Module.  Different States
/// allow for different actions to be taken.
/// 
/// The following table gives important information as to the states of the registers
/// given a state.
/// 
/// Mode       | PWR_UP | PRIM_RX | CE | FIFO state
/// ------------------------------------------------------------
/// RX         |    1   |    2    |  1 | -----------------------
/// ------------------------------------------------------------
/// TX         |    1   |    0    |  1 | Empty TX FIFOs
/// ------------------------------------------------------------
/// Standby 2  |    1   |    0    |  1 | TX FIFO Empty
/// ------------------------------------------------------------
/// Standby 1  |    1   |    -    |  0 | No packet transmission
/// ------------------------------------------------------------
/// Power Down |    0   |    -    |  - |  ----------------------
/// ------------------------------------------------------------
/// 
/// State Transitions:
/// 
/// Transition              | Maximum Time | Minimum Time
/// -----------------------------------------------------
/// Power Down -> Standby 1 | 1.5ms        | 150us
/// -----------------------------------------------------
/// Standby -> TX/RX Mode   | 130us        | -----
/// -----------------------------------------------------
/// CE High                 | -----        | 10us
/// -----------------------------------------------------
/// CE High -> CSN Low      | -----        | 4us
/// -----------------------------------------------------
#[derive(PartialEq, Debug, Clone, Copy)]
pub enum State {
    // Registers are maintained, but the module cannot do anything until
    // turned on
    PowerDown,
    // Low power usage waiting to switch to rx or tx modes
    Standby1,
    // More power than Standby 1, but lower startup time and
    // occurs when TX FIFO is empty (automatically sends TX info if put in
    // the FIFO)
    Standby2,
    // Actively receiving data (receives until FIFOS are full)
    Rx,
    // Actively transmitting data (stays here until finished sending packets
    // then it goes to Standby2)
    Tx,
}