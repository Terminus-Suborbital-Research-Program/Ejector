use bin_packets::{phases::EjectorPhase, types::UnixTimestampMillis};

/// State machine for the Ejector, holds the current phase
pub struct EjectorStateMachine {
    phase: EjectorPhase,
}

impl EjectorStateMachine {
    /// Ejector always enters the Standby phase first
    pub fn new() -> Self {
        Self {
            phase: EjectorPhase::Standby,
        }
    }

    /// Mabye transition to the next phase, depending on conditions
    /// returns the number of ms we should wait before trying to transition again
    pub fn transition(&mut self) -> u64 {
        let (phase, time) = match self.phase {
            // Standby only moves into ejection if explicitly commanded, so
            // we don't model that here
            EjectorPhase::Standby => (EjectorPhase::Standby, 0),

            EjectorPhase::Ejection => (EjectorPhase::Hold, 5000),

            EjectorPhase::Hold => (EjectorPhase::Hold, 10000),
        };
        self.phase = phase;
        time
    }
}
