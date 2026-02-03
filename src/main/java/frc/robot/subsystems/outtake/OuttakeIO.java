package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface OuttakeIO extends Subsystem{
    @AutoLog
    public static class OuttakeIOInputs {
        /**The current speed in rpm of the outtake motor */
        public double outtakeMotorSpeed = 0.0;
        /** */
        public double outtakeMotorVoltage = 0.0;
        public double hoodAngle = 0.0;
    }

    public void outtake(double speed);

    public void updateInputs(OuttakeIOInputs inputs);

    public void startFlywheel();

    public void stopFlywheel();
    
    public void setIntakeVoltage(Voltage voltage);
}
