package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public double appliedVoltage = 0;
    }
    
    // Updates the logged inputs
    public default void updateInputs (IndexerIOInputs inputs) {}

    // Runs the indexer at the specified voltage
    
    public default void setVoltage (double voltage) {}
}