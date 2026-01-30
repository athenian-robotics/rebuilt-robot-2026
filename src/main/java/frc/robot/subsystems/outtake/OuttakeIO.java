package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface OuttakeIO extends Subsystem{
    @AutoLog
    public static class OuttakeIOLogs {
        /**The current speed in rpm of the outtake motor */
        public double outtakeMotorSpeed = 0.0;
        /** */
        public double outtakeMotorVoltage = 0.0;
        public double hoodAngle = 0.0;
    }

    public final TalonFX outtakeMotor = null;

    public Command outtake(double speed);

    public void updateLogs(OuttakeIOLogs logs);
}
