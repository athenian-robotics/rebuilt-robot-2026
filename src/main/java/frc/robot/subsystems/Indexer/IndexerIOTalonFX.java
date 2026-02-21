package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX motor;
    private double appliedVoltage = 0;

    public IndexerIOTalonFX() {
        motor = new TalonFX(IndexerConstants.MOTOR_ID, new CANBus(CANConstants.CANIVORE_NAME));
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.appliedVoltage = appliedVoltage;
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Indexer/Voltage", voltage);
        appliedVoltage = voltage;
        motor.setControl(new VoltageOut(voltage));
    }
}
