package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake.BasicControlState;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX armMotor = new TalonFX(IntakeConstants.ARM_ID, new CANBus(CANConstants.CANIVORE_NAME));
    private final TalonFX wheelMotor = new TalonFX(IntakeConstants.WHEEL_ID, new CANBus(CANConstants.CANIVORE_NAME));

    private BasicControlState basicControlState = BasicControlState.STOPPED;

    private double sysIdVoltage = 0.0;
    private double setpointRotations = 0.0;

    public IntakeIOTalonFX() {
        // Sets gear ratios
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(1 / IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        // Applies configs
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration()
                .withFeedback(feedbackConfigs);

        // Sets up for more sophisticated feedback/feedforward control methods (that we don't currently use)
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kP = IntakeConstants.INTAKE_kP;
        slot0Configs.kI = IntakeConstants.INTAKE_kI;
        slot0Configs.kD = IntakeConstants.INTAKE_kD;
        slot0Configs.kS = IntakeConstants.INTAKE_kS;
        slot0Configs.kV = IntakeConstants.INTAKE_kV;
        slot0Configs.kA = IntakeConstants.INTAKE_kA;
        slot0Configs.kG = IntakeConstants.INTAKE_kG;

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.INTAKE_MAX_ACCELERATION;

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonFXConfigs.Voltage.PeakForwardVoltage = IntakeConstants.MAX_ARM_VOLTAGE;
        talonFXConfigs.Voltage.PeakReverseVoltage = -IntakeConstants.MAX_ARM_VOLTAGE;

        armMotor.getConfigurator().apply(talonFXConfigs);
        armMotor.setPosition(IntakeConstants.ARM_STARTING_POSITION_ROT);
        armMotor.getConfigurator().apply(Constants.MECHANISM_CURRENT_LIMITS);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        wheelMotor.getConfigurator().apply(Constants.MECHANISM_CURRENT_LIMITS);
    }

    /** Update the set of loggable inputs */
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.armMotorVoltage_Volts = armMotor.getMotorVoltage().getValueAsDouble();
        inputs.armMotorCurrent_Amps = armMotor.getTorqueCurrent().getValueAsDouble();
        inputs.armRotations_Rotations = armMotor.getPosition().getValueAsDouble();
        inputs.armMotorRotations_Rotations = inputs.armRotations_Rotations
                / IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;

        inputs.wheelMotorVoltage_Volts = wheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.wheelMotorCurrent_Amps = wheelMotor.getTorqueCurrent().getValueAsDouble();
        inputs.wheelMotorVelocity_RotationsPerSecond = wheelMotor.getVelocity().getValueAsDouble();
        inputs.setpoint_Rotations = setpointRotations;
        inputs.closedLoopReferenceRot = armMotor.getClosedLoopReference().getValueAsDouble();
        // if (-Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble()) +
        // IntakeConstants.FULL_RETRACTION_DEGREES <
        // IntakeConstants.BASIC_CONTROL_TOLERANCE_DEG) {
        // usingBasicControl = false;
        // }
    }

    @Override
    public void periodic() {
        // If robot is undergoing sysID, ignore everything that would normally happen and simply sysID.
        if (sysIdVoltage != 0) {
            armMotor.setControl(new VoltageOut(sysIdVoltage));
            return;
        }

        // Check the basicControl state and set motor control based on direction
        switch (basicControlState) {
            case FORWARD:
                armMotor.setControl(
                    new VoltageOut(IntakeConstants.BASIC_CONTROL_FORWARD_VOLTS)
                );
                break;
            
                case BACKWARD:
                armMotor.setControl(
                    new VoltageOut(IntakeConstants.BASIC_CONTROL_BACKWARD_VOLTS)
                );
                break;
            
            case STOPPED:
                armMotor.set(0);
                break;
            
            default:
                break;
        }
    }

    @Override
    public void goWithBasicControl(BasicControlState direction) {
        basicControlState = direction;
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
        log.motor("arm").angularPosition(armMotor.getPosition().getValue())
                .angularVelocity(armMotor.getVelocity().getValue()).voltage(Volts.of(sysIdVoltage));
    }

    @Override
    public void setAngle(double angleDeg) {
      basicControlState = BasicControlState.DISABLED;
      armMotor.setControl(new MotionMagicVoltage(angleDeg / 360.0));
      setpointRotations = angleDeg / 360.0;
    }

    @Override
    public double getTargetDeg() {
      return setpointRotations * 360;
    }

    @Override
    @AutoLogOutput
    public boolean atSetpoint() {
      return armMotor.getMotionMagicAtTarget().getValue();
    }
}
