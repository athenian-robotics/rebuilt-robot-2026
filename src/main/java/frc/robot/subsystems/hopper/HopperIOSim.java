package frc.robot.subsystems.hopper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants.HopperConstants;

// TODO fully implement class
public class HopperIOSim implements HopperIO {
    
    DCMotor m = DCMotor.getNeo550(1);
    
    
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final SparkMaxSim simMotor = new SparkMaxSim(hopperMotor, m);
    private final SparkClosedLoopController pidController = hopperMotor.getClosedLoopController();
    private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
    
    
    /** Update the set of loggable inputs */
    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperMotor_Volts =  hopperMotor.getBusVoltage();
        inputs.hopperMotor_Amps = hopperMotor.getOutputCurrent();
        inputs.hopperExtension_Rotations = hopperEncoder.getPosition();
        inputs.hopperExtension_Inches = inputs.hopperExtension_Rotations / HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION;
    }
   
    @Override
    public void goToPosition(double position_inches){
        pidController.setSetpoint(position_inches * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION, SparkBase.ControlType.kMAXMotionPositionControl);
    }
    public boolean atSetpoint(){
        return pidController.isAtSetpoint();
    }
}
