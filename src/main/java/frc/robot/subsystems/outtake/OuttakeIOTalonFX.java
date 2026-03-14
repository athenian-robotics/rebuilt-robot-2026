package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeIOTalonFX extends SubsystemBase implements OuttakeIO {
  private final TalonFX leadShooter, followShooter, middleWheel, angleChanger, starWheelMotor;
  private final BangBangController flywheelController;
  private double flywheelSetpointRPS = 0.0;

  private double currentAngleDeg = 0.0;
  private double currentAngularVelocityDegPerSecond = 0.0;
  private double targetDistanceMeters = 0.0;

  private DoubleEntry hoodAngleDegEntry;

  private double sysIdVoltage = 0.0;

  public OuttakeIOTalonFX() {
    super();
    starWheelMotor = new TalonFX(OuttakeConstants.STAR_WHEEL_MOTOR, new CANBus(CANConstants.CANIVORE_NAME));

    hoodAngleDegEntry = NetworkTableInstance.getDefault().getDoubleTopic("/Outtake/HoodAngleDeg").getEntry(OuttakeConstants.MINIMUM_HOOD_ANGLE_DEG);

    followShooter = new TalonFX(OuttakeConstants.LEFT_SHOOTER_MOTOR, new CANBus(CANConstants.CANIVORE_NAME));
    leadShooter = new TalonFX(OuttakeConstants.RIGHT_SHOOTER_MOTOR, new CANBus(CANConstants.CANIVORE_NAME));
    flywheelController = new BangBangController();

    followShooter.setControl(
        new Follower(OuttakeConstants.RIGHT_SHOOTER_MOTOR, MotorAlignmentValue.Opposed));
    
    leadShooter.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)); // To make re-spinning up faster
    middleWheel = new TalonFX(OuttakeConstants.MIDDLE_WHEEL_MOTOR, new CANBus(CANConstants.CANIVORE_NAME));

    angleChanger = new TalonFX(OuttakeConstants.ANGLE_CHANGER_MOTOR, new CANBus(CANConstants.CANIVORE_NAME));
 

    // Represents the starting position of the hood
    angleChanger.setPosition(OuttakeConstants.ANGLE_CHANGER_STARTING_ANGLE_ROTATIONS);
    currentAngleDeg = OuttakeConstants.ANGLE_CHANGER_STARTING_ANGLE_ROTATIONS * 360;

    // applies feedforward and feedback constants
    Slot0Configs angleChangerControl = new Slot0Configs();
    angleChangerControl.kP = OuttakeConstants.HOOD_ANGLE_KP;
    angleChangerControl.kD = OuttakeConstants.HOOD_ANGLE_KD;
    angleChangerControl.kS = OuttakeConstants.HOOD_ANGLE_KS;
    angleChangerControl.kV = OuttakeConstants.HOOD_ANGLE_KV;
    angleChangerControl.kA = OuttakeConstants.HOOD_ANGLE_KA;
    angleChanger.getConfigurator().apply(angleChangerControl);

    // Applies our maximum hood angular velocity and angular acceleration
    MotionMagicConfigs angleChangerMotionProfile = new MotionMagicConfigs();
    angleChangerMotionProfile.MotionMagicCruiseVelocity = OuttakeConstants.HOOD_ANGLE_CRUISE_VELOCITY_RPS;
    angleChangerMotionProfile.MotionMagicAcceleration = OuttakeConstants.HOOD_ANGLE_MAX_ACCELERATION_RPSPS;
    angleChanger.getConfigurator().apply(angleChangerMotionProfile);

    // Applies gear ratios into the code
    angleChanger.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(1.0/OuttakeConstants.ANGLE_CHANGER_GEAR_RATIO));
    
    middleWheel.getConfigurator().apply(Constants.CURRENT_LIMITS);
    angleChanger.getConfigurator().apply(Constants.CURRENT_LIMITS);
    starWheelMotor.getConfigurator().apply(Constants.CURRENT_LIMITS);
  }

  public void periodic() {
    // Update the current angle by getting the motor position and multiplying by gear ratio
    currentAngleDeg = angleChanger.getPosition().getValue().in(Degrees);
    // Update the current angular velocity by getting the motor velocity and multiplying by gear ratio
    currentAngularVelocityDegPerSecond = angleChanger.getVelocity().getValue().in(DegreesPerSecond);
  
    // if (sysIdVoltage != 0.0) {
    //   angleChanger.setControl(new VoltageOut(sysIdVoltage));
    // } else {
    //   angleChanger.setControl(new VoltageOut(0));
    // }
    
    // If we're trying to spin up
    if (flywheelSetpointRPS != 0.0) {
      // Apply 12 volts if we're under the target speed, otherwise 0 volts
      leadShooter.setControl(new VoltageOut(-OuttakeConstants.FLYWHEEL_VOLTS * flywheelController.calculate(-leadShooter.getVelocity().getValue().in(RotationsPerSecond), flywheelSetpointRPS)));
    } else {
      leadShooter.set(0);
    }
  }
  

  /**
   * Calculates how to aim the shooter to hit a target.
   *
   * @param currentPosition the position to aim from
   * @param targetPosition the position to aim at
   * @return the angle in degrees required to shoot into the hub from its location as an optional double
   */
  public OptionalDouble calculateAngle(Translation2d currentPosition, Translation2d targetPosition) {
    // The horizontal part of the distance between the robot and the target
    targetDistanceMeters = currentPosition.getDistance(targetPosition);

    // Based on a curve fit from google sheets
    // If inaccurate, we should add more data points and maybe use a more sophisticated control
    // Should probably be sinusoidal somehow
    double angle = -6.6 + 17.2 * targetDistanceMeters - 2.02 * Math.pow(targetDistanceMeters, 2);

    return OptionalDouble.of(angle);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    // Inputs for IO logging
    inputs.currentHoodAngleDegrees = currentAngleDeg;
    inputs.currentAngularVelocityDegPerSecond = currentAngularVelocityDegPerSecond;
    inputs.targetHoodAngleDegrees = angleChanger.getClosedLoopReference().getValueAsDouble() * 360.0;
    inputs.targetDistanceFeet = Units.metersToFeet(targetDistanceMeters);
    inputs.angleChangerVoltage = angleChanger.getMotorVoltage().getValueAsDouble();
    inputs.indexerVoltage = starWheelMotor.getMotorVoltage().getValueAsDouble();
    inputs.setpoint_RPS = flywheelSetpointRPS;
    inputs.flywheel_RPS = leadShooter.getVelocity().getValue().in(RotationsPerSecond);
    inputs.armEncoderAngle_rot = angleChanger.getPosition().getValueAsDouble();
    inputs.middleWheelVoltage = middleWheel.getMotorVoltage().getValueAsDouble();
    inputs.flywheelVoltage = (Math.abs(leadShooter.getMotorVoltage().getValueAsDouble()) + Math.abs(followShooter.getMotorVoltage().getValueAsDouble()))/2.0;
    inputs.flywheelSpunUp = isSpunUp();
  }

  public void startFlywheel() {
    flywheelSetpointRPS = OuttakeConstants.FLYWHEEL_VELOCITY_RPS;
    System.out.println(leadShooter.getMotorVoltage().getValue());
  }

  public void stopFlywheel() {
    flywheelSetpointRPS = 0.0;
  }

  @Override
  public void setMiddleWheelVoltage(double voltage) {
    middleWheel.setControl(new VoltageOut(voltage));
    Logger.recordOutput("Outtake/MiddleWheelVoltage", voltage);
  }

  public void setStarWheelVoltage(double voltage) {
    starWheelMotor.setControl(new VoltageOut(voltage));
    Logger.recordOutput("Outtake/StarWheelVoltage", voltage);
  }

  public void setAngleAtTarget(Translation2d currentPosition) {
    if (DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Red) {
      calculateAngle(currentPosition, OuttakeConstants.HUB_POSITION_RED)
        .ifPresent(this::setAngle);
    } else {
      calculateAngle(currentPosition, OuttakeConstants.HUB_POSITION_BLUE)
        .ifPresent(this::setAngle);
    }
  }

  public void setAngle(double angleDegrees) {
    if (OuttakeConstants.MAXIMUM_HOOD_ANGLE_DEG < angleDegrees 
     || OuttakeConstants.MINIMUM_HOOD_ANGLE_DEG > angleDegrees) {
      System.out.println("setAngle was passed an invalid angle");
      return;
    }
    
    hoodAngleDegEntry.set(angleDegrees);

    MotionMagicDutyCycle magic = new MotionMagicDutyCycle(angleDegrees/360.0);

    angleChanger.setControl(magic);
  }

  @Override
  public void setAngleFromNT() {
    setAngle(hoodAngleDegEntry.get());
  }

  @Override
  public void runSysId(double voltage) {
    sysIdVoltage = voltage;
  }

  @Override
  public void sysIDLog(SysIdRoutineLog log) {
      log.motor("angle").voltage(Volts.of(sysIdVoltage)).angularPosition(Degrees.of(currentAngleDeg)).angularVelocity(DegreesPerSecond.of(currentAngularVelocityDegPerSecond));
  }

  @Override
  public boolean isSpunUp() {
    return Math.abs(Math.abs(leadShooter.getVelocity().getValueAsDouble()) - flywheelSetpointRPS) < OuttakeConstants.FLYWHEEL_MAX_ERROR_RPS;
  }
}
