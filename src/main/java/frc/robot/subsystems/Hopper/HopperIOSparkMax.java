package frc.robot.subsystems.Hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.HopperConstants;


public class HopperIOSparkMax implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
    
    private double hopperVoltage_Volts = 0;
    
    private final SimpleMotorFeedforward hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
    
    
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperExtension_rotations = hopperEncoder.getPosition();
        inputs.hopperExtension_inches = rotationsToLength(inputs.hopperExtension_rotations);
        inputs.hopperExtensionVelocity_inchesPerSecond = hopperEncoder.getVelocity();
        inputs.hopperMotor_Volts = hopperMotor.getBusVoltage();
        inputs.hopperMotor_Amps = hopperMotor.getAppliedOutput();
        
    }
    private double linearToAngularVelocity(double lin_velocity){
            return (HopperConstants.WINCH_ROTATIONS / (HopperConstants.WINCH_DIAMETER_INCHES * Math.PI)) * lin_velocity;
    }
    public double velocityToVoltage(double velocity){
        return hopperFeedforward.calculate(velocity);
    }

   
    public void setVoltage(double volts){
        hopperMotor.setVoltage(volts);
    }

    public double rotationsToLength(double rotations){
        return ((HopperConstants.WINCH_DIAMETER_INCHES * Math.PI) / HopperConstants.WINCH_ROTATIONS) * rotations;
    }
    @Override
    public void moveToLength(double length_inches) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToLength'");
    }
    
  

   
}
