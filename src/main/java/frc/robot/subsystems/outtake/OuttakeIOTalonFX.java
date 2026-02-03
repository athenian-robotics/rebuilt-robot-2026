package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.outtake.OuttakeIO;

public class OuttakeIOTalonFX extends SubsystemBase implements OuttakeIO {
  private final TalonFX rightShooter, leftShooter, intake, angleChanger;
  private final OuttakeIOInputs logs;

  public OuttakeIOTalonFX() {
    super();
    logs = new OuttakeIOInputs();
    leftShooter = new TalonFX(OuttakeConstants.LEFT_SHOOTER_MOTOR);
    rightShooter = new TalonFX(OuttakeConstants.RIGHT_SHOOTER_MOTOR);
    leftShooter.setControl(new Follower(OuttakeConstants.RIGHT_SHOOTER_MOTOR, MotorAlignmentValue.Opposed));
    intake = new TalonFX(OuttakeConstants.INTAKE_MOTOR);
    angleChanger = new TalonFX(OuttakeConstants.ANGLE_CHANGER_MOTOR);
  }

  /**
   * Sets the angle of the hood. 
   * @param degrees The number of degrees to turn to, from <x>º to <y>º 
   */
  public Command setHoodAngle(double degrees) {
    // logs.hoodAngle.set(degrees);
    double target = 0;
    return new InstantCommand(() -> angleChanger.setPosition(target), this);
  }

  public Command setHoodAngle() {
    return setHoodAngle(0.0);
  }
  
  /**
   * Calculates how to aim for the hub. 
   * @return the angle required to shoot into the hub from its location
   */
  public double calculateAngle() {
    return 0.0;
  }

  public Command intake() {
    return new StartEndCommand(() -> intake.set(0.1), () -> intake.set(0.0), this);
  }

  public Command outtake(double speed) {
    return null;
  }

  @Override
  public void updateInputs(OuttakeIOInputs logs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateLogs'");
  }

  public void startFlywheel() {
    rightShooter.setControl(new VoltageOut(Volts.of(12)));
  }

  public void stopFlywheel() {
    rightShooter.set(0.0);
  }

  public void setIntakeVoltage(Voltage voltage) {
    intake.setControl(new VoltageOut(voltage));
  }
}
