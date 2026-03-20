package frc.robot.subsystems.outtake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO extends Subsystem {
  @AutoLog
  public static class OuttakeIOInputs {
    /** The actual angle of the hood, in degrees, representing hood angle compared to horizontal with ccw+ */
    public double currentHoodAngleDegrees = 0.0;
    /** The target angle of the hood, in degrees, representing hood angle compared to horizontal with ccw+ */
    public double targetHoodAngleDegrees = 0.0;
    /** The current angular velocity of the hood in degrees per second */
    public double currentAngularVelocityDegPerSecond = 0.0;
    /** Distance to the target in feet */
    public double armEncoderAngle_rot = 0.0;
    public double targetDistanceFeet = 0.0;
    public double angleChangerVoltage = 0.0;
    public double indexerVoltage = 0.0;
    public double setpoint_RPS = 0.0;
    public double flywheel_RPS = 0.0;
    public double middleWheelVoltage = 0.0;
    public double flywheelVoltage = 0.0;
    public boolean flywheelSpunUp = false;
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

  /** Causes the star wheel to spin using specified voltage */
  public default void setStarWheelVoltage(double voltage){};

  /** Sets the target shot angle, which the hood will constantly move towards, to be targeting the target */
  public default void setAngleAtTarget(Translation2d currentPosition) {};

  /** Sets the target shot angle, which the hood will constantly move towards, measured ccw+ from horiontal */
  public default void setAngle(double angleDegrees) {};

  /** Sets the target angle based on the current network table value at /Outtake/HoodAngleDeg */
  public default void setAngleFromNT () {};

  /**
   * Applies a set amount of voltage to the angle changer motor for sysID purposes
   * @param voltage the amount of voltage to apply
   */
  public default void runSysId(double voltage) {};

  /**
   * This method should be called in Outtake.java's periodic method, causing it to be run when the subsystem periodic is run.
   */
  public default void periodic () {}

  public default void sysIDLog (SysIdRoutineLog log) {}

  /**
   * Checks whether the robot's flywheel is within OuttakeConstants.MAX_ERROR_RPS of the target.
   * @return true if so, false otherwise
   */
  public default boolean isSpunUp () {return false;}

  /**
   * Set the rpm of the flywheel to an arbitrary value. 
   * @param rpm
   */
  public default void setRMP (double rpm) {}
}
