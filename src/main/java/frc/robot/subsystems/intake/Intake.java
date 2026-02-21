package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Config sysIdConfig = new Config(null,null,null);
  Mechanism sysIdMechanism = new Mechanism((volts) -> io.runSysId(volts.in(Volts)), null, this);
  private SysIdRoutine sysId = new SysIdRoutine(sysIdConfig, sysIdMechanism);
  
  
  public Intake(IntakeIO io){
    this.io = io;
  }
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
  /**fully extends the intake from {@value IntakeConstants#FULL_RETRACTION_DEGREES} degrees 
   * to {@value IntakeConstants#FULL_EXTENSION_DEGREES} degrees*/
  public void fullyExtend() {
    System.out.println("full extension");
    io.goToPosition(IntakeConstants.FULL_EXTENSION_DEGREES); // position feedback loop
  }
  /**fully retracts the intake from {@value IntakeConstants#FULL_EXTENSION_DEGREES} degrees 
   * to {@value IntakeConstants#FULL_RETRACTION_DEGREES} degrees*/
  public void fullyRetract(){
    System.out.println("full retraction");
    io.goToPosition(IntakeConstants.FULL_RETRACTION_DEGREES);
  }

  public void wiggleUp() {
    System.out.println("wiggle up");
    io.goToPosition(IntakeConstants.MAX_WIGGLE_DEGREES);
     // velocity feedback loop
  }
  public boolean atSetpoint(){
    return io.atSetpoint();
  }

  public Command runIntake() {
    return Commands.startEnd(io::startIntake, io::stopIntake, this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return run(() -> io.runSysId(0.0))
    .withTimeout(1.0)
    .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return run(() -> io.runSysId(0.0))
    .withTimeout(1.0)
    .andThen(sysId.dynamic(direction));
  }
}
