package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;
public class climb extends SubsystemBase{
    private climbIO io;
    private climbIOInputsAutoLogged inputs;

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("climb", inputs);
    }

    public void extend(){
        io.goToPosition(ClimbConstants.FULL_EXTENSION);
    }
    public void retract(){
        io.goToPosition(ClimbConstants.FULL_RETRACTION);
    }
}
