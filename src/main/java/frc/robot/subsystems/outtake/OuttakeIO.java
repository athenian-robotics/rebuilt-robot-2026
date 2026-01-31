package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
    @AutoLog
    public static class OuttakeIOInputs {
        /**The current speed in radians per second of the outtake motor */
        public double outtakeMotorRadPerSec = 0.0;
        /**The current voltage of the outtake motor */
        public double outtakeMotorVoltage = 0.0;
        /**The current angle of the hood in degrees from the horizontal */
        public double hoodAngle = 0.0;
    }

    public void updateLogs(OuttakeIOInputs inputs);
}
