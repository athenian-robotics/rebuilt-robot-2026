package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO{
    private final DCMotorSim simMotorModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(IntakeConstants.INTAKE_kV, IntakeConstants.INTAKE_kA), DCMotor.getKrakenX60(1));


    private final TalonFX armMotor = new TalonFX(IntakeConstants.TALON_ID);
    private final TalonFXSimState simMotor = new TalonFXSimState(armMotor);
    private double setpoint = 0;


    public IntakeIOSim(){
        var talonFXConfigs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = IntakeConstants.INTAKE_kP;
    slot0Configs.kI = IntakeConstants.INTAKE_kI;
    slot0Configs.kD = IntakeConstants.INTAKE_kD;
    slot0Configs.kS = IntakeConstants.INTAKE_kS;
    slot0Configs.kV = IntakeConstants.INTAKE_kV;
    slot0Configs.kA = IntakeConstants.INTAKE_kA;
    
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicAcceleration = 0;
    motionMagicConfigs.MotionMagicJerk = 0;
    armMotor.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
   
        simMotor.setSupplyVoltage(12);


    inputs.armMotor_Voltage_Volts = simMotor.getMotorVoltage();
    inputs.armMotor_Current_Amps = simMotor.getTorqueCurrent();
    inputs.armMotorRotations_Rotations = armMotor.getPosition().getValueAsDouble();
    inputs.armRotations_Rotations = 
        inputs.armMotorRotations_Rotations * IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;
    inputs.armSetpoint = armMotor.getClosedLoopReference().getValueAsDouble();
    

    simMotorModel.setInputVoltage(inputs.armMotor_Voltage_Volts);
    simMotorModel.update(0.02);

    simMotorModel.getAngularPosition();
    simMotorModel.getAngularVelocity();

    simMotor.setRawRotorPosition(simMotorModel.getAngularPosition());
    simMotor.setRotorVelocity(simMotorModel.getAngularVelocity());
    }

   @Override
  public void goToPosition(double armRotations_Degrees) {
 //   setpoint = armRotations_Degrees / IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;
    setpoint = armRotations_Degrees;
    System.out.println(setpoint);
    System.out.println(IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS);
    final MotionMagicVoltage m_request = new MotionMagicVoltage(armMotor.getPosition().getValueAsDouble());

    
        armMotor.setControl(m_request.withPosition(Units.degreesToRotations(setpoint)));
    

  }
  @Override
  public boolean atSetpoint(){
    return armMotor.getMotionMagicAtTarget().getValue();
  }
}
