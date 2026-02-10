package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

public class HopperIntakeCommands{
    public static Command startingExtension(Hopper hopper, Intake intake){
        return Commands.runOnce(hopper::partial, hopper, intake)
        .andThen(Commands.waitUntil(hopper::atSetpoint))
        .andThen(Commands.runOnce(hopper::full).alongWith(Commands.runOnce(intake::fullyExtend)));
    }
   public static Command hopperRetract(Hopper hopper, Intake intake){
        return Commands.runOnce(intake::fullyExtend).andThen(Commands.waitUntil(intake::atSetpoint)
        .andThen(Commands.runOnce(hopper::retract)));
   }
   public static Command hopperExtend(Hopper hopper, Intake intake){
        return Commands.runOnce(intake::fullyExtend).andThen(Commands.waitUntil(intake::atSetpoint)
        .andThen(Commands.runOnce(hopper::full)));
   }
   public static Command fuck(Hopper hopper){
     return Commands.runOnce((hopper::full), hopper);
   }
}