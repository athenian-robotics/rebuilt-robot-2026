package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.climb;

public class ClimbCommands {
    public Command climbExtend(climb climb){
        return Commands.runOnce(climb::extend, climb);
    }
    public Command climbRetract(climb climb){
        return Commands.runOnce(climb::retract, climb);
    }
}

