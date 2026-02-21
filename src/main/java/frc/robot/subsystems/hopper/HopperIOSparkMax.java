package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

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
        inputs.hopperSetpoint_Inches = pidController.getSetpoint()/HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION;
        inputs.hopperMAXMotionSetpoint_Inches = pidController.getMAXMotionSetpointPosition();
        inputs.motorTemp = hopperMotor.getMotorTemperature();
        
    }
   
    @Override
    public void goToPosition(double position_inches){
        // pidController.setSetpoint(position_inches * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION, SparkBase.ControlType.kMAXMotionPositionControl);
        //pidController.setSetpoint(position_inches, SparkBase.ControlType.kMAXMotionPositionControl);
        pidController.setSetpoint(position_inches, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        // System.out.println("STATUS".concat(err.toString()));

    }
    @Override
    public boolean atSetpoint(){
        return pidController.isAtSetpoint();
    }
    public double getGoal(){
        return pidController.getSetpoint();
    }
    public void setVoltage(){
        this.setVoltage();
    }
}
