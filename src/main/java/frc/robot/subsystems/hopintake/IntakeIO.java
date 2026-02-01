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

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double kV;
        public double kS;
        public Angle intakeRotation = Degrees.of(0);
        public Voltage intakeArmVoltage = Volts.of(0);
        public Voltage intakeCompressionVoltage = Volts.of(0);
        public SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(kS, kV);
    }
    
    /** Update the set of loggable inputs */
    public void intakeArmMotorSpeed(AngularVelocity velocity);
    public void intakeCompressionMotorSpeed(AngularVelocity velocity);


        

        // Constants (TODO move)
        

        
    
}
