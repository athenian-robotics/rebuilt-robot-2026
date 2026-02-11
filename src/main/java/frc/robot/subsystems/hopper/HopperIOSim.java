package frc.robot.subsystems.hopper;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants.HopperConstants;

// TODO fully implement class
public class HopperIOSim implements HopperIO {
    SparkMax motor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    DCMotor m = DCMotor.getNeo550(1);
    //motor.getClosedLoo
    
    private final SparkMaxSim hopperMotor = new SparkMaxSim(motor, m);
    private final SparkClosedLoopController pidController = motor.getClosedLoopController();

    
    
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperMotor_Volts =  hopperMotor.getBusVoltage();
        //hopperMotor.
        
    }
    public double positionToRotations(double position){
       return ((position * HopperConstants.HOPPER_WINCH_GEAR_RATIO) / HopperConstants.HOPPER_WINCH_CIRCUMFRENCE);

    }
    public void goToPosition(double position_inches){
        pidController.setSetpoint(positionToRotations(position_inches), SparkBase.ControlType.kMAXMotionPositionControl);
    }

    @Override
    public boolean atSetpoint() {
        return pidController.isAtSetpoint();
    }
    @Override
    public double getGoal() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getGoal'");
    }
    @Override
    public void setVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }
   
}
