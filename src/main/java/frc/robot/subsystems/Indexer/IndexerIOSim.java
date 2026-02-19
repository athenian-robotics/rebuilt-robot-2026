package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.IndexerConstants;

public class IndexerIOSim implements IndexerIO {
    private double appliedVoltage = 0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.appliedVoltage = appliedVoltage;
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Indexer/Voltage", voltage);
        appliedVoltage = voltage;
    }
}
