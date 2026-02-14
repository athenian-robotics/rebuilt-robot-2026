package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO{
    private final TalonFX armMotor = new TalonFX(IntakeConstants.TALON_ID);
    private final PIDController 
    pidController = new PIDController(IntakeConstants.INTAKE_kP, 
    IntakeConstants.INTAKE_kI, 
    IntakeConstants.INTAKE_kD);

    private final TalonFXSimState simMotor = new TalonFXSimState(armMotor);
    public IntakeIOSim(){
        var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = IntakeConstants.INTAKE_kP;
    slot0Configs.kI = IntakeConstants.INTAKE_kI;
    slot0Configs.kD = IntakeConstants.INTAKE_kD;
    slot0Configs.kS = IntakeConstants.INTAKE_kS;
    slot0Configs.kV = IntakeConstants.INTAKE_kV;
    
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicAcceleration = 0;
    motionMagicConfigs.MotionMagicJerk = 0;
    armMotor.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        
    inputs.armMotor_Voltage_Volts = simMotor.getMotorVoltage();
    inputs.armMotor_Current_Amps = simMotor.getTorqueCurrent();
    inputs.armMotorRotations_Rotations = armMotor.getPosition().getValueAsDouble();
    inputs.armRotations_Rotations = 
        inputs.armMotorRotations_Rotations * IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;
    }

    @Override
    public void goToPosition(double rotations) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'goToPosition'");
    }

    @Override
    public boolean atSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'atSetpoint'");
    }
}
