package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private SysIdRoutine sysId;
  
  public Intake(IntakeIO io){
    this.io = io;
    
    Config sysIdConfig = new Config(Volts.per(Seconds).of(1), Volts.of(3), Seconds.of(5));
    Mechanism sysIdMechanism = new Mechanism((volts) -> io.runSysId(volts.in(Volts)), io::sysIDLog, this);

    sysId = new SysIdRoutine(sysIdConfig, sysIdMechanism);
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
  public Command openHopper() {
    System.out.println("opening hopper with intake");
    return Commands.runOnce(() -> io.goToPosition(IntakeConstants.HOPPER_OPEN_DEGREES), this);
  }
  /**fully retracts the intake from {@value IntakeConstants#FULL_EXTENSION_DEGREES} degrees 
   * to {@value IntakeConstants#FULL_RETRACTION_DEGREES} degrees*/
  public Command fullyRetract(){
    System.out.println("full retraction");
    return Commands.runOnce(() -> io.goToPosition(IntakeConstants.FULL_RETRACTION_DEGREES), this);
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

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return run(() -> io.runSysId(0.0))
          .withTimeout(1.0)
          .andThen(sysId.quasistatic(direction));
      }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return run(() -> io.runSysId(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
