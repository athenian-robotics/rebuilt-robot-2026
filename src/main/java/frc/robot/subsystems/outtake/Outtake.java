package frc.robot.subsystems.outtake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.OuttakeConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs;
  private final SysIdRoutine sysId;

  public Outtake(OuttakeIO io) {
    this.io = io;
    inputs = new OuttakeIOInputsAutoLogged();

    Config sysIdConfig = new Config(Volts.per(Seconds).of(.3), Volts.of(1), Seconds.of(5));
    Mechanism sysIdMechanism = new Mechanism((volts) -> io.runSysId(volts.in(Volts)), io::sysIDLog, this);

    sysId = new SysIdRoutine(sysIdConfig, sysIdMechanism);
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs("Outtake", inputs);
  }

  /**
   * Starts running the flywheel. Will take a few seconds to get up to speed.
   * @return An instant command to start the flywheels
   */
  public Command startFlywheel(){
    return new InstantCommand(io::startFlywheel);
  }

  /**
   * Stops running the flywheel. Flywheel will be allowed to coast to preserve robot battery
   * @return An instant command to stop powering the flywheels.
   */
  public Command stopFlywheel() {
    return new InstantCommand(io::stopFlywheel);
  }

  /**
   * Lowers the hood to fit under the trench, and to shoot when right against the hub.
   * @return An instant command to set the hood into motion
   */
  public Command lowerHood() {
    return new InstantCommand(() -> io.setAngle(OuttakeConstants.MINIMUM_HOOD_ANGLE_DEG));
  }

  /**
   * Causes the middle and star wheels to spin such that nearby fuel enters the shooter
   * @return A continuous command that keeps the wheels running
   */
  public Command sendBallsToShooter() {
    return new StartEndCommand(
        () -> {
          System.out.println("things!");
          io.setMiddleWheelVoltage(-OuttakeConstants.MIDDLE_WHEEL_TO_SHOOTER_VOLTS);
          io.setStarWheelVoltage(-OuttakeConstants.STAR_WHEEL_TO_SHOOTER_VOLTS);
        },
        () -> {
          io.setMiddleWheelVoltage(0);
          io.setStarWheelVoltage(0);
        },
        this);
  }

  /**
   * Causes the middle and star wheels to spin such that nearby fuel is deposited onto the ground
   * @return A continuous command that keeps the wheels running
   */
  public Command groundOuttake() {
    return new StartEndCommand(
        () -> {
            io.setMiddleWheelVoltage(-OuttakeConstants.MIDDLE_WHEEL_TO_GROUND_VOLTS);
            io.setStarWheelVoltage(-OuttakeConstants.STAR_WHEEL_TO_GROUND_VOLTS);
        },
        () -> {
            io.setMiddleWheelVoltage(0);
            io.setStarWheelVoltage(0);
        },
        this);
  }

  /**
   * Maps joystick Y input to hood angle across the configured min/max range.
   *
   * @param joystickY joystick axis in [-1, 1]
   * @return command that continuously tracks joystick position
   */
  // public Command aimWithJoystick(DoubleSupplier joystickY) {
  //   return Commands.run(() -> {
  //         // Invert axis so pushing forward increases the command value.
  //         double invertedY = -joystickY.getAsDouble();
  //         double normalized = (invertedY + 1.0) / 2.0;
  //         double angleRange =
  //             OuttakeConstants.MINIMUM_HOOD_ANGLE_DEG - OuttakeConstants.MAXIMUM_HOOD_ANGLE_DEG;
  //         double targetAngle = OuttakeConstants.MAXIMUM_HOOD_ANGLE_DEG + (normalized * angleRange);
  //         io.setAngle(targetAngle);
  //       }, this);
  // }

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

    /**
     * Sets the target angle based on the current network table value at /Outtake/HoodAngleDeg
     * To set the network table value, type a value in elastic and press enter while the robot is disabled
     * @return An instant command to set the hood into motion
     */
    public Command toNTAngle () {
      return Commands.runOnce(io::setAngleFromNT, this);
    }

    /**
     * Causes the hood to aim at the hub, based on solely the distance between the centers of the bot and hub.
     * @param currentPosition The current robot position
     * @return An instant command to set the hood into motion
     */
    public Command aimAtTarget (Supplier<Translation2d> currentPosition) {
      return Commands.runOnce(() -> io.setAngleAtTarget(currentPosition.get()));
    }

    /**
     * Causes the hood to go to a specified angle
     * @param angleDegrees Supplies the angle the hood should go to. Is sampled once when this command is executed
     * @return An instant command to set the hood into motion
     */
    public Command setAngle (DoubleSupplier angleDegrees) {
      return Commands.runOnce(() -> io.setAngle(angleDegrees.getAsDouble()));
    }

    /**
     * Checks whether the robot's flywheel is close (within FlywheelConstants.MAX_ERROR_RPS) to the target rps
     * @return true if it is, false if it isn't
     */
    public boolean isSpunUp () {
      return io.isSpunUp();
    }

    /**
     * Add trim to the hood angle. Doesn't apply immediatly. 
     * @param trimDeg
     * @return An instant command to add trim
     */
    public Command addTrim (double trimDeg) {
        return Commands.runOnce(() -> io.addTrim(trimDeg));
    }
}
