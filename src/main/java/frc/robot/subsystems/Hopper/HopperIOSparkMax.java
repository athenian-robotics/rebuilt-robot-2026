package frc.robot.subsystems.Hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.HopperConstants;


public class HopperIOSparkMax implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final SparkClosedLoopController pidController = hopperMotor.getClosedLoopController();
    private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
    
    
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperMotor_Volts =  hopperMotor.getBusVoltage();
        inputs.hopperMotor_Amps = hopperMotor.getOutputCurrent();
        
    }
    public void goToPosition(double position_inches){
        pidController.setSetpoint(position_inches, SparkBase.ControlType.kMAXMotionPositionControl);
    }
   
}
