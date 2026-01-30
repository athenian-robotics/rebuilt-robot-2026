package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.outtake.OuttakeIO;

public class Shooter extends SubsystemBase implements OuttakeIO {
  private final TalonFX rightShooter, leftShooter, intake, angleChanger;
  private final OuttakeIOLogs logs;

  public Shooter() {
    super();
    logs = new OuttakeIOLogs();
    rightShooter = new TalonFX(-1);
    leftShooter = new TalonFX(-1);
    leftShooter.setControl(new Follower(-1, MotorAlignmentValue.Opposed));
    intake = new TalonFX(-1);
    angleChanger = new TalonFX(-1);
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
  public void updateLogs(OuttakeIOLogs logs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateLogs'");
  }
}
