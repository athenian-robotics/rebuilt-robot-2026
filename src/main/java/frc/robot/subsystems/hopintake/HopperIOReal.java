package frc.robot.subsystems.hopintake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.generated.TunerConstants;

public class HopperIOReal implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
    
    private final SimpleMotorFeedforward hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
    
    private 
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperExtension = Rotations.of(hopperEncoder.getPosition());

    }
    private void setHopperMotorSpeed(AngularVelocity velocity) {
        hopperMotor.setVoltage(hopperFeedforward.calculate(velocity.in(DegreesPerSecond)));
    }

    public void doSomething() {}
}
