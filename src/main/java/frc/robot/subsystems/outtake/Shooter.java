package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIO.OuttakeIOInputs;
import frc.robot.Constants.OuttakeConstants;

public class Shooter extends SubsystemBase {
  /**This is the leader of the leftShooter. It's gearing is 144:1 */
  private final TalonFX rightShooter;
  /**This follows the right shooter */
  private final TalonFX leftShooter;
  private final TalonFX intake, angleChanger, starWheel;
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged logs;

  public Shooter() {
    super();
    rightShooter = new TalonFX(OuttakeConstants.RIGHT_SHOOTER_ID);
    leftShooter = new TalonFX(OuttakeConstants.LEFT_SHOOTER_ID);
    leftShooter.setControl(new Follower(OuttakeConstants.RIGHT_SHOOTER_ID, MotorAlignmentValue.Opposed));
    intake = new TalonFX(OuttakeConstants.INTAKE_MOTOR_ID);
    angleChanger = new TalonFX(OuttakeConstants.ANGLE_CHANGER_ID);
    starWheel = new TalonFX(OuttakeConstants.STAR_WHEEL_ID);
    io = new OuttakeIOShooter(intake);
    logs = new OuttakeIOInputsAutoLogged();
  }

  /**
   * Sets the angle of the hood. 
   * @param degrees The number of degrees to turn to, from {@value OuttakeConstants#MIN_HOOD_ANGLE_DEGREES}º to {@value OuttakeConstants#MAX_HOOD_ANGLE_DEGREES}º 
   */
  public Command setHoodAngle(double degrees) {
    degrees = Math.max(OuttakeConstants.MIN_HOOD_ANGLE_DEGREES, Math.min(OuttakeConstants.MAX_HOOD_ANGLE_DEGREES, degrees));
    double target = 0;
    return new InstantCommand(() -> angleChanger.setPosition(target), this);
  }
  /**Sets the angle of the hood to {@link #calculateAngle} */
  public Command setHoodAngle() {
    return setHoodAngle(calculateAngle());
  }
  
  /**
   * Calculates how to aim for the hub. 
   * @return the angle required to shoot into the hub from its location
   */
  public double calculateAngle() {
    return new Saathvik().mechanics(new Pose2d());
  }

  public Command intake() {
    return new StartEndCommand(
      () -> {intake.set(OuttakeConstants.INTAKE_SPEED); starWheel.set(OuttakeConstants.INTAKE_SPEED);}, 
      () -> {intake.set(0); starWheel.set(0);}, 
      this);
  }

  public Command outtake() {
    return new StartEndCommand(() -> rightShooter.set(OuttakeConstants.OUTTAKE_SPEED), () -> rightShooter.set(0), this);
  }
}
