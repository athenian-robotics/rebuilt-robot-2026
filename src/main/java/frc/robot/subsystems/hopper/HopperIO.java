package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.HopperConstants;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs{
        public double hopperExtension_Rotations = 0;
        public double hopperExtension_Inches = (hopperExtension_Rotations  / HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
       // public double hopperExtensionVelocity_inchesPerSecond = 0;
        public double hopperExtensionVelocity_inchesPerSecond = 0;
        public double hopperMotor_Volts = 0;
        public double hopperMotor_Amps = 0;
        public double hopperSetpoint_Inches = 0;
        //public double hopperFeedforward = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
        public double hopperMAXMotionSetpoint_Inches;
        public double motorTemp;
    }
    /** Update the set of loggable inputs */
    public default void updateInputs(HopperIOInputs inputs){};
    /**
     * Goes the hopper to position 
     * @param position in inches
     */
    public default void goToPosition(double position){};
    
    public default boolean atSetpoint(){return false;};
    public default double getGoal(){return 0.0;};
    public default void setVoltage(){};
    

        

        // Constants (TODO move)
}
