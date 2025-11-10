package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drive extends SubsystemBase {
  private SwerveDrive drivetrain;

  // -- Alerts --
  // private final Alert noSwerveConfigFoundAlert = new Alert(getName(), "No YAGSL
  // swerve configuration found.", AlertType.kError);

  public Drive() {
    try {
      drivetrain =
          new SwerveParser(Filesystem.getDeployDirectory())
              .createSwerveDrive(DrivetrainConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond));
    } catch (Exception e) {
      throw new RuntimeException("Unable to load subsystem", e);
    }

    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
  }
}
