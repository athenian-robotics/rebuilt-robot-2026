package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake.BasicControlState;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double armMotorVoltage_Volts = 0;
        public double armMotorCurrent_Amps = 0;
        public double armMotorRotations_Rotations = 0;
        public double armRotations_Rotations = armMotorRotations_Rotations * IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;

        public double wheelMotorVoltage_Volts = 0;
        public double wheelMotorCurrent_Amps = 0;
        public double wheelMotorVelocity_RotationsPerSecond = 0;
        public double setpoint_Rotations = IntakeConstants.ARM_STARTING_POSITION_ROT;
        public double closedLoopReferenceRot = IntakeConstants.ARM_STARTING_POSITION_ROT;
    }

    public default void periodic() {
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    };

    public default void startIntake() {
    };

    public default void stopIntake() {
    }

    public default void runSysId(double voltage) {
    };

    public default void sysIDLog(SysIdRoutineLog log) {
    };

    public default void goWithBasicControl(BasicControlState direction) {
    }

    public default void setAngle(double angleDeg) {}

    public default double getTargetDeg() {return 0;}
}
