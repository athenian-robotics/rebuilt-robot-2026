package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ClimbConstants;
public class climbIOTalonFX implements climbIO{
      private final TalonFX climbMotor = new TalonFX(ClimbConstants.TALON_ID);
      public climbIOTalonFX(){
          var talonFXConfigs = new TalonFXConfiguration();
            var slot0Configs = talonFXConfigs.Slot0;
            slot0Configs.kP = ClimbConstants.CLIMB_kP;
            slot0Configs.kI = ClimbConstants.CLIMB_kI;
            slot0Configs.kD = ClimbConstants.CLIMB_kD;
            slot0Configs.kS = ClimbConstants.CLIMB_kS;
            slot0Configs.kV = ClimbConstants.CLIMB_kV;
            slot0Configs.kA = ClimbConstants.CLIMB_kA;
            slot0Configs.kG = ClimbConstants.CLIMB_kG;
    
            var motionMagicConfigs = talonFXConfigs.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.CLIMB_CRUISE_VELOCITY;
            motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.CLIMB_MAX_ACCELERATION;
            motionMagicConfigs.MotionMagicJerk = ClimbConstants.CLIMB_MAX_ALLOWED_PROFILER_ERROR;
    
            climbMotor.getConfigurator().apply(talonFXConfigs);
      }
    @Override
    public void updateInputs(climbIOInputs inputs) {
        inputs.climbMotorVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
        inputs.climbMotorExtension_rotations = climbMotor.getRotorPosition().getValueAsDouble();
        inputs.climbMotorExtension_inches = 
        inputs.climbMotorExtension_rotations * ClimbConstants.ROTATIONS_TO_LENGTH;
        inputs.climbMotorVelocity_rotationsPerSecond = climbMotor.getRotorVelocity().getValueAsDouble();
        inputs.climbMotorVelocity_inchesPerSecond = 
        inputs.climbMotorVelocity_rotationsPerSecond * ClimbConstants.ROTATIONS_TO_LENGTH;
    }

    @Override
    public void goToPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'goToPosition'");
    }
    
}