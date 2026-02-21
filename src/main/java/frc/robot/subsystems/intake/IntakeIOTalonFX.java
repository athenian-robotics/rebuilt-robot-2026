package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;



public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX armMotor = new TalonFX(IntakeConstants.ARM_ID);
  private final TalonFX wheelMotor = new TalonFX(IntakeConstants.WHEEL_ID);
  private final CANcoder hello = new CANcoder(67);


  // private final PIDController feedback = new PIDController
  // (IntakeConstants.INTAKE_kP, 
  // IntakeConstants.INTAKE_kI, 
  // IntakeConstants.INTAKE_kD);
  // private final ArmFeedforward feedforward = new ArmFeedforward
  // (IntakeConstants.INTAKE_kS,
  // IntakeConstants.INTAKE_kG,
  // IntakeConstants.INTAKE_kV,
  // IntakeConstants.INTAKE_kA);

  public IntakeIOTalonFX(){
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
      .withSensorToMechanismRatio(IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS)
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

    slot0Configs.GravityArmPositionOffset = 0;
    
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.INTAKE_MAX_ACCELERATION;

    armMotor.getConfigurator().apply(talonFXConfigs);
  }

  /** Update the set of loggable inputs */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armMotorVoltage_Volts = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.armMotorCurrent_Amps = armMotor.getTorqueCurrent().getValueAsDouble();
    inputs.armMotorRotations_Rotations = armMotor.getPosition().getValueAsDouble();
    inputs.armRotations_Rotations = 
        inputs.armMotorRotations_Rotations * IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;

    inputs.wheelMotorVoltage_Volts = wheelMotor.getMotorVoltage().getValueAsDouble();
    inputs.wheelMotorCurrent_Amps = wheelMotor.getTorqueCurrent().getValueAsDouble();
    inputs.wheelMotorVelocity_RotationsPerSecond = wheelMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void goToPosition(double armRotations_Degrees) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    armMotor.setControl(m_request.withPosition(Units.degreesToRotations(armRotations_Degrees)  / IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS));
  }

  @Override
  public void startIntake() {
    wheelMotor.setControl(new VoltageOut(IntakeConstants.WHEEL_VOLTAGE));
  }

  @Override
  public void stopIntake() {
    wheelMotor.setControl(new VoltageOut(0));
  }
}
