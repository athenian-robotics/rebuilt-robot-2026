package frc.robot.subsystems.hopintake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface HopintakeIO {
    @AutoLog
    public static class HopintakeIOInputs{
        public static class Hopper {
            public double kV;
            public double kS;
            public Distance hopperExtension = Inches.of(0);
            public Voltage hopperVoltage = Volts.of(0);
            public SimpleMotorFeedforward hopperFeedforward = new SimpleMotorFeedforward(kS, kV);
        }
        public static class Intake{
            public double kV;
            public double kS;
            public Angle intakeRotation = Degrees.of(0);
            public Voltage intakeArmVoltage = Volts.of(0);
            public Voltage intakeCompressionVoltage = Volts.of(0);
            public SimpleMotorFeedforward hopperFeedforward = new SimpleMotorFeedforward(kS, kV);
        }
    }
    /** Update the set of loggable inputs */
    public void updateInputs(HopintakeIOInputs inputs);
    public void hopperMotorSpeed(AngularVelocity velocity);
    public void intakeArmMotorSpeed(AngularVelocity velocity);
    public void intakeCompressionMotorSpeed(AngularVelocity velocity);


        

        // Constants (TODO move)
        

        
    
}
