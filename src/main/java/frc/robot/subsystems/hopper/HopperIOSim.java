package frc.robot.subsystems.hopper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants.HopperConstants;

public class HopperIOSim implements HopperIO {
    
    DCMotor m = DCMotor.getNeo550(1);
    
    
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final SparkMaxSim simMotor = new SparkMaxSim(hopperMotor, m);
    private final PIDController pidController = new PIDController(0, 0, 0);
    private final SparkRelativeEncoderSim simEncoder = simMotor.getRelativeEncoderSim();
    
    
    
    /** Update the set of loggable inputs */
    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperMotor_Volts =  simMotor.getBusVoltage();
        inputs.hopperMotor_Amps = simMotor.getMotorCurrent();
        inputs.hopperExtension_Rotations = simEncoder.getPosition();
        inputs.hopperExtension_Inches = inputs.hopperExtension_Rotations / HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION;
    }
   
    @Override
    public void goToPosition(double position_inches){
        double voltage = pidController.calculate(simEncoder.getPosition(), position_inches * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
        simMotor.setBusVoltage(voltage);
    }
    public boolean atSetpoint(){
        return pidController.atSetpoint();
    }
}
