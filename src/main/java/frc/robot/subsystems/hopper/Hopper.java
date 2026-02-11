package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private HopperIO io;
    private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    private double setpoint = 0;

    public Hopper(HopperIO io){
        this.io = io;
    
    }
    @Override
    public void periodic() {
        double target = io.getGoal();
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
        io.goToPosition(target);
    }

    /**
     * Retracts the hopper to fully retracted
     */
    public void retract() {
        System.out.println("fucking hell");
        io.goToPosition(HopperConstants.HOPPER_RETRACTED * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
    }
    

    /**
     * Moves the hopper to {@value HopperConstants#HOPPER_PARTIAL} inches
     */
    public void partial() {
        System.out.println("good god");
        setpoint = HopperConstants.HOPPER_PARTIAL * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION;
        //io.goToPosition();
    }
    /**
     * Moves the hopper to max extension of {@value HopperConstants#HOPPER_FULL}
     */
    public void full() {
        System.out.println("oh no");
        io.goToPosition(HopperConstants.HOPPER_FULL * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
    }
    public boolean atSetpoint(){
        return io.atSetpoint();
    }

}
