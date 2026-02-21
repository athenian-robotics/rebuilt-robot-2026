package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double armMotorVoltage_Volts = 0;
    public double armMotorCurrent_Amps = 0;
    public double armMotorRotations_Rotations = 0;
    public double armRotations_Rotations =
        armMotorRotations_Rotations * IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;
    
    public double wheelMotorVoltage_Volts = 0;
    public double wheelMotorCurrent_Amps = 0;
    public double wheelMotorVelocity_RotationsPerSecond = 0;
  }

  public void updateInputs(IntakeIOInputs inputs);
  public void goToPosition(double rotations);
  public void startIntake();
  public void stopIntake();
  public default void runSysId(double voltage) {};
}
