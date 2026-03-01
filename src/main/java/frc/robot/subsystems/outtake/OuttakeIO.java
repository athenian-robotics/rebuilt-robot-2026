package frc.robot.subsystems.outtake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO extends Subsystem {
  @AutoLog
  public static class OuttakeIOInputs {
    /** The actual angle of the hood, in degrees, representing launch angle compared to horizontal with ccw+ */
    public double currentShotAngleDegrees = 0.0;
    /** The target angle of the hood, in degrees, representing launch angle compared to horizontal with ccw+ */
    public double targetShotAngleDegrees = 0.0;
    /** The current angular velocity of the hood in degrees per second */
    public double currentAngularVelocityDegPerSecond = 0.0;
    /** Distance to the target in feet */
    public double targetDistanceFeet = 0.0;
    public double angleChangerVoltage = 0.0;
    public double indexerVoltage = 0.0;
    public double setpoint_RPS = 0.0;
    public double flywheel_RPS = 0.0;
  }

  /** Updates logs; util for AdvantageScope
   * @param inputs inputs from previous iteration
   */
  public default void updateInputs(OuttakeIOInputs inputs) {};

  /** Causes the flywheel to start spinning up */
  public default void startFlywheel() {};

  /** Causes the flywheel to coast */
  public default void stopFlywheel() {};

  /** Causes the "middle" wheel to spin using specified voltage */
  public default void setMiddleWheelVoltage(double voltage) {};

  /** Causes the indexer AND star wheel to spin using specified voltage */

  public default void setIndexerVoltage(double voltage){};

  /** Sets the target shot angle, which the hood will constantly move towards, to be targeting the target */
  public default void setAngleAtTarget(Translation2d currentPosition) {};

  /** Sets the target shot angle, which the hood will constantly move towards, measured ccw+ from horiontal */
  public default void setAngle(double angleDegrees) {};

  /** Sets the target angle based on the current network table value at /Outtake/HoodAngleDeg */
  public default void setAngleFromNT () {};

  public default void runSysId(double voltage) {};

  public default void stopAngleChanging() {}

  public default void periodic () {}

}
