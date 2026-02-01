package frc.robot.subsystems.hopintake;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.hopintake.HopintakeIO.HopintakeIOInputs;

public class HopintakeIOSim implements HopintakeIO {
        private double x;
    /** Update the set of loggable inputs */
    @Override
    public void updateInputs(HopintakeIOInputs inputs) {
        
    }

    @Override
    public void hopperMotorSpeed(AngularVelocity velocity) {

    }
    @Override
    public void intakeArmMotorSpeed(AngularVelocity velocity) {

    }
    @Override
    public void intakeCompressionMotorSpeed(AngularVelocity velocity) {
        
    }
}
