package frc.robot.subsystems.Hopper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.hopintake.HopperIOInputsAutoLogged;


public class HopperIOSparkMax implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
    
    private final SimpleMotorFeedforward hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
    
    
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperExtension_rotations = hopperEncoder.getPosition();
        inputs.hopperExtension_inches = rotationsToLength(inputs.hopperExtension_rotations);
    }
    private double linearToAngularVelocity(double lin_velocity){
            return (HopperConstants.WINCH_GEARBOX_GEAR_RATIO / (HopperConstants.WINCH_DIAMETER_INCHES * Math.PI)) * lin_velocity;
    }
    private void setHopperMotorSpeed(double lin_velocity) {
        double velocity = linearToAngularVelocity(lin_velocity);
        hopperMotor.setVoltage(hopperFeedforward.calculate(velocity));
    }
    @Override
    public void moveToLength(double length) {
        // TODO Auto-generated method stub
        
    }
    public double rotationsToLength(double rotations){
        return ((HopperConstants.WINCH_DIAMETER_INCHES * Math.PI) / HopperConstants.WINCH_GEARBOX_GEAR_RATIO) * rotations;
    }
    @Override
    public void updateInputs(HopperIOInputsAutoLogged inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

   
}
