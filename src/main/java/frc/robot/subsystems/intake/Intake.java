package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  
  public Intake(IntakeIO io){
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
  /**fully extends the intake from {@value IntakeConstants#FULL_RETRACTION_DEGREES} degrees 
   * to {@value IntakeConstants#FULL_EXTENSION_DEGREES} degrees
 * @return */
  public void fullyExtend() {
    System.out.println("full extension");
    io.goToPosition(IntakeConstants.FULL_EXTENSION_DEGREES); // position feedback loop
  }
  /**
   * Opens the intake arm far enough to push the hopper out before the hopper subsystem
   * is removed, matching the desired {@link IntakeConstants#HOPPER_OPEN_DEGREES}.
   */
  public void openHopper() {
    System.out.println("opening hopper with intake");
    io.goToPosition(IntakeConstants.HOPPER_OPEN_DEGREES);
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
}
