package frc.robot.subsystems.Hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.HopperConstants;


public class HopperIOSim implements HopperIO {
    SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    DCMotor m = DCMotor.getNeo550(1);
    
    private final SparkMaxSim hopperMotor = new SparkMaxSim(motor, m);
    private final SparkClosedLoopController pidController = motor.getClosedLoopController();
    private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
    
    
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperMotor_Volts =  hopperMotor.getBusVoltage();
        inputs.hopperMotor_Amps = hopperMotor.getOutputCurrent();
        
    }
    public double positionToRotations(double position){
       return ((position * HopperConstants.HOPPER_WINCH_GEAR_RATIO) / HopperConstants.HOPPER_WINCH_CIRCUMFRENCE);

    }
    public void goToPosition(double position_inches){
        pidController.setSetpoint(positionToRotations(position_inches), SparkBase.ControlType.kMAXMotionPositionControl);
    }
   
}
