package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HopperConstants;


public class HopperIOSparkMax implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final SparkClosedLoopController pidController = hopperMotor.getClosedLoopController();
    private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
    
    public HopperIOSparkMax() {
        SparkMaxConfig cfg = new SparkMaxConfig();

        cfg.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        cfg.closedLoop.maxMotion
            .cruiseVelocity(HopperConstants.HOPPER_CRUISE_VELOCITY)
            .maxAcceleration(HopperConstants.HOPPER_MAX_ACCELERATION)
            .allowedProfileError(HopperConstants.HOPPER_MAX_ALLOWED_PROFILER_ERROR);
        
        cfg.closedLoop.feedForward
            .kV(HopperConstants.HOPPER_kV)
            .kA(HopperConstants.HOPPER_kA)
            .kS(HopperConstants.HOPPER_kS);
        cfg.closedLoop
            .pid(HopperConstants.HOPPER_kP, HopperConstants.HOPPER_kI, HopperConstants.HOPPER_kD);

        hopperMotor.configure(cfg, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }
    /** Update the set of loggable inputs */
    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperMotor_Volts =  hopperMotor.getBusVoltage();
        inputs.hopperMotor_Amps = hopperMotor.getOutputCurrent();
        inputs.hopperExtension_Rotations = hopperEncoder.getPosition();
        inputs.hopperExtension_Inches = inputs.hopperExtension_Rotations / HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION;
        inputs.hopperSetpoint_Inches = getGoal();
    }
   
    @Override
    public void goToPosition(double position_inches){
        pidController.setSetpoint(position_inches * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION, SparkBase.ControlType.kMAXMotionPositionControl);
      
    }
    @Override
    public boolean atSetpoint(){
        return pidController.isAtSetpoint();
    }

    @Override
    public double getGoal() {
        return pidController.getSetpoint();
    }

   
}
