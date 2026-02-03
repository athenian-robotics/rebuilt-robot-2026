package frc.robot.subsystems.hopintake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HopperConstants;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs{
        public Distance hopperExtension = Inches.of(0);
        public LinearVelocity hopperExtensionVelocity = InchesPerSecond.of(0);
        public Voltage hopperVoltage = Volts.of(0);
        public Amperage currentAmperage = Amps.of(0);
        public SimpleMotorFeedforward hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
        
    }
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputs inputs);
   // public void hopperMotorSpeed(AngularVelocity velocity);
    public void moveToLength(Distance length);
    
    

    

        

        // Constants (TODO move)
}
