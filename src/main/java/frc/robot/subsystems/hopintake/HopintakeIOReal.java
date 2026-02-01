package frc.robot.subsystems.hopintake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

public class HopintakeIOReal implements HopintakeIO {
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final TalonFX motor = 
    private final SimpleMotorFeedforward hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
    

    /** Update the set of loggable inputs */
    public void updateInputs(HopintakeIOInputs inputs) {
        
    }
    public void hopperMotorSpeed(AngularVelocity velocity) {
        motor.setVoltage(hopperFeedforward.calculate(velocity.in(DegreesPerSecond)));
    }
    public void intakeArmMotorSpeed(AngularVelocity velocity) {

    }
    public void intakeCompressionMotorSpeed(AngularVelocity velocity) {

    }

}
