package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class IndexerIOSim implements IndexerIO {
    private Voltage appliedVoltage = Volts.of(0);

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.appliedVoltage = appliedVoltage;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        appliedVoltage = voltage;
    }
    
}
