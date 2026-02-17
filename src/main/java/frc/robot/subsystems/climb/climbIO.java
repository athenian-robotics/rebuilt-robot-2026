package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;


public interface climbIO {
    @AutoLog
    public static class climbIOInputs{
        public double climbMotorVoltage = 0;
        public double climbMotorExtension_inches = 0;
        public double climbMotorExtension_rotations = 0;
        public double climbMotorVelocity_rotationsPerSecond = 0;
        public double climbMotorVelocity_inchesPerSecond = 0;
    }
     /** Update the set of loggable inputs */
    public void updateInputs(climbIOInputs inputs);
    /**
     * Goes the climber to position 
     * @param position in inches
     */
    public void goToPosition(double position);
    

    

    
}
