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

    Config sysIdConfig = new Config(Volts.per(Seconds).of(.15), Volts.of(0.5), Seconds.of(5),
            (state) -> Logger.recordOutput("Outtake/SysIdState", state.toString()));
    Mechanism sysIdMechanism = new Mechanism((volts) -> io.runSysId(volts.in(Volts)), null, this);

    sysId = new SysIdRoutine(sysIdConfig, sysIdMechanism);
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs("Outtake", inputs);
  }

  /**
   * @return Returns a InstantCommand that starts spinning the flywheel and angles the hood.
   * @param currentPosition The current position of the robot center on the field
   */
  public Command enterShootMode(Translation2d currentPosition) {
    return new InstantCommand(io::startFlywheel).andThen(new InstantCommand(() -> io.setAngleAtTarget(currentPosition)));
  }

  /**
   * Starts the flywheel and angles the hood according to the parameter
   * @param shotAngleDeg The angle at which the ball will exit the shooter ccw+ from horizontal
   * @return The command
   */
  public Command enterShootMode(double shotAngleDeg) {
    return new InstantCommand(io::startFlywheel).andThen(new InstantCommand(() -> io.setAngle(shotAngleDeg)));
  }
  public Command startFlywheel(){
    return new InstantCommand(io::startFlywheel);
  }

  public Command lowerHood() {
    return new InstantCommand(() -> io.setAngle(OuttakeConstants.MAXIMUM_SHOT_ANGLE_DEG));
  }

  public Command stopFlywheel() {
    return new InstantCommand(io::stopFlywheel);
  }

  public Command sendBallsToShooter() {
    return new StartEndCommand(
        () -> {
          io.setMiddleWheelVoltage(-OuttakeConstants.MIDDLE_WHEEL_TO_SHOOTER_VOLTS);
          io.setStarWheelVoltage(-OuttakeConstants.STAR_WHEEL_TO_SHOOTER_VOLTS);
        },
        () -> {
          io.setMiddleWheelVoltage(0);
          io.setStarWheelVoltage(0);
        },
        this);
  }

  public Command groundOuttake() {
    return new StartEndCommand(
        () -> {
            io.setMiddleWheelVoltage(OuttakeConstants.MIDDLE_WHEEL_TO_GROUND_VOLTS);
            io.setStarWheelVoltage(OuttakeConstants.STAR_WHEEL_TO_GROUND_VOLTS);
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
  public Command aimWithJoystick(DoubleSupplier joystickY) {
    return Commands.run(() -> {
          // Invert axis so pushing forward increases the command value.
          double invertedY = -joystickY.getAsDouble();
          double normalized = (invertedY + 1.0) / 2.0;
          double angleRange =
              OuttakeConstants.MAXIMUM_SHOT_ANGLE_DEG - OuttakeConstants.MINIMUM_SHOT_ANGLE_DEG;
          double targetAngle = OuttakeConstants.MINIMUM_SHOT_ANGLE_DEG + (normalized * angleRange);
          io.setAngle(targetAngle);
        }, this);
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
