package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakeConstants;



public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX armMotor = new TalonFX(IntakeConstants.ARM_ID, new CANBus(CANConstants.CANIVORE_NAME));
  private final TalonFX wheelMotor = new TalonFX(IntakeConstants.WHEEL_ID, new CANBus(CANConstants.CANIVORE_NAME));

  private int usingBasicControl = 0;

  private double sysIdVoltage = 0.0;
  private double setpoint_Rotations = 0.00;

  // private final PIDController feedback = new PIDController
  // (IntakeConstants.INTAKE_kP, 
  // IntakeConstants.INTAKE_kI, 
  // IntakeConstants.INTAKE_kD); new ArmFeedforward
  // (IntakeConstants.INTAKE_kS,
  // IntakeConstants.INTAKE_kG,
  // IntakeConstants.INTAKE_kV,
  // IntakeConstants.INTAKE_kA);

  public IntakeIOTalonFX(){
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
      .withSensorToMechanismRatio(1/IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
   
    var talonFXConfigs = new TalonFXConfiguration()
      .withFeedback(feedbackConfigs);
    
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = IntakeConstants.INTAKE_kP;
    slot0Configs.kI = IntakeConstants.INTAKE_kI;
    slot0Configs.kD = IntakeConstants.INTAKE_kD;
    slot0Configs.kS = IntakeConstants.INTAKE_kS;
    slot0Configs.kV = IntakeConstants.INTAKE_kV;
    slot0Configs.kG = IntakeConstants.INTAKE_kG;
    
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.INTAKE_MAX_ACCELERATION;

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfigs.Voltage.PeakForwardVoltage = IntakeConstants.MAX_ARM_VOLTAGE;
    talonFXConfigs.Voltage.PeakReverseVoltage = -IntakeConstants.MAX_ARM_VOLTAGE;

    armMotor.getConfigurator().apply(talonFXConfigs);
    armMotor.setPosition(IntakeConstants.ARM_STARTING_POSITION_ROT);
    armMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(70).withSupplyCurrentLowerLimit(35).withSupplyCurrentLowerTime(1).withSupplyCurrentLimitEnable(true));
    wheelMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(70).withSupplyCurrentLowerLimit(35).withSupplyCurrentLowerTime(1).withSupplyCurrentLimitEnable(true));
  }

  /** Update the set of loggable inputs */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armMotorVoltage_Volts = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.armMotorCurrent_Amps = armMotor.getTorqueCurrent().getValueAsDouble();
    inputs.armRotations_Rotations = armMotor.getPosition().getValueAsDouble();
    inputs.armMotorRotations_Rotations = 
        inputs.armMotorRotations_Rotations / IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;

    inputs.wheelMotorVoltage_Volts = wheelMotor.getMotorVoltage().getValueAsDouble();
    inputs.wheelMotorCurrent_Amps = wheelMotor.getTorqueCurrent().getValueAsDouble();
    inputs.wheelMotorVelocity_RotationsPerSecond = wheelMotor.getVelocity().getValueAsDouble();
    inputs.setpoint_Rotations = setpoint_Rotations;


    if (usingBasicControl > 0) {
      // if (-Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble()) + IntakeConstants.FULL_RETRACTION_DEGREES < IntakeConstants.BASIC_CONTROL_TOLERANCE_DEG) {
      //   usingBasicControl = false;
      // }
      armMotor.setControl(new VoltageOut(IntakeConstants.BASIC_CONTROL_VOLTS));
    } else if(usingBasicControl < 0) {
      armMotor.set(-0.4);
    } else {
      armMotor.set(0);
    }
  }

  @Override
  public void goWithBasicControl(int direction) {
    usingBasicControl = direction;
  }

  @Override
  public void goToPosition(double armRotations_Degrees) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    armMotor.setControl(m_request.withPosition(Units.degreesToRotations(armRotations_Degrees)));
    setpoint_Rotations = Units.degreesToRotations(armRotations_Degrees);
  }
  public boolean atSetpoint(){
    return armMotor.getMotionMagicAtTarget().getValue();
  }

  @Override
  public void startIntake() {
    wheelMotor.setControl(new VoltageOut(IntakeConstants.WHEEL_VOLTAGE));
  }

  @Override
  public void stopIntake() {
    wheelMotor.setControl(new VoltageOut(0));
  }

  @Override
  public void runSysId(double voltage) {
    sysIdVoltage = voltage;
  }

  @Override
  public void sysIDLog(SysIdRoutineLog log) {
    log.motor("arm").angularPosition(armMotor.getPosition().getValue()).angularVelocity(armMotor.getVelocity().getValue()).voltage(Volts.of(sysIdVoltage));
  }
}
