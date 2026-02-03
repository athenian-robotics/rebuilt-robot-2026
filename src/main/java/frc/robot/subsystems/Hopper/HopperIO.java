package frc.robot.subsystems.Hopper;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.hopintake.HopperIOInputsAutoLogged;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs{
        public double hopperExtension_rotations = 0;
        public double hopperExtension_inches = 0;
        public double hopperExtensionVelocity = 0;
        public double hopperVoltage = 0;
        public double currentAmperage = 0;
        //public double hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
        
    }
    /** Update the set of loggable inputs */
    public void updateInputs(HopperIOInputsAutoLogged inputs);
   // public void hopperMotorSpeed(AngularVelocity velocity);
    public void moveToLength(double length_inches);
    
    

    

        

        // Constants (TODO move)
}
