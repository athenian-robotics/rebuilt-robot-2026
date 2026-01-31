package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class OuttakeIOShooter implements OuttakeIO {
    private final StatusSignal<AngularVelocity> outtakeRotationSpeed;
    private final StatusSignal<Voltage> outtakeAppliedVolts;

    public OuttakeIOShooter(TalonFX outtakeMotor) {
        outtakeRotationSpeed = outtakeMotor.getVelocity();
        outtakeAppliedVolts = outtakeMotor.getMotorVoltage();
    }

    @Override
    public void updateLogs(OuttakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(outtakeRotationSpeed,outtakeAppliedVolts);
        inputs.outtakeMotorRadPerSec = outtakeRotationSpeed.getValueAsDouble();
        inputs.outtakeMotorVoltage = outtakeAppliedVolts.getValueAsDouble();
    }

    
}