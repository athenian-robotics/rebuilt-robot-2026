package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

public class HopperIntakeCommands{
//     public Command startingExtension(Hopper hopper, Intake intake){
//         return Commands.runOnce(hopper::partial)
//         .andThen(Commands.waitUntil(hopper::atSetpoint))
//         .andThen(Commands.runOnce(hopper::full).alongWith(Commands.runOnce(intake::fullyExtend)));
//     }
   public Command hopperRetract(Hopper hopper, Intake intake){
        return Commands.runOnce(intake::fullyExtend).andThen(Commands.waitUntil(intake::atSetpoint)
        .andThen(Commands.runOnce(hopper::retract)));
   }
   public Command hopperExtend(Hopper hopper, Intake intake){
        return Commands.runOnce(intake::fullyExtend).andThen(Commands.waitUntil(intake::atSetpoint)
        .andThen(Commands.runOnce(hopper::full)));
   }
   public static Command goFull(Hopper hopper){
     return Commands.runOnce(hopper::full);
   }
   public static Command goPartial(Hopper hopper){
     return Commands.runOnce(hopper::partial);
   }
   public static Command goRetract(Hopper hopper){
     return Commands.runOnce(hopper::retract);
   }
   public static Command intakeGoFull(Intake intake){
    return Commands.runOnce(intake::fullyExtend);
   }
   public static Command intakeRetract(Intake intake){
    return Commands.runOnce(intake::fullyRetract);
   }
  

    
}