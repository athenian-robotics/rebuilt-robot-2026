package frc.robot.subsystems.Hopper;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase{
            
    private HopperIO io;
    private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }
    public double positionToTime(double position){
        return (position / HopperConstants.HOPPER_VELOCITY_INCHESPERSECOND);
    }
    public double velocityToVoltage(double velocity){
        return io.velocityToVoltage(velocity);
    }

    public void retractedToPartial(){
        io.moveToLength(HopperConstants.SETPOINT_PARTIALLY_EXTENDED_INCHES);
    }
    public void partialToFull(){
        io.moveToLength(HopperConstants.SETPOINT_EXTENDED_INCHES);
    }
    public void fullToRetracted(){
        io.moveToLength(HopperConstants.SETPOINT_RETRACTED_INCHES);
    }


}
