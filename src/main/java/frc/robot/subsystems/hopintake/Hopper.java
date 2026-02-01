package frc.robot.subsystems.hopintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Revolutions;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveCommandsConstants;
import frc.robot.commands.DriveCommands;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Hopper extends SubsystemBase{
            
    private HopperIO io;
    private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void retractedToPartial(){
        io.moveToLength(SETPOINT_PARTIALLY_EXTENDED);
    }
    public void partialToFull(){
            io.moveToLength(SETPOINT_EXTENDED);
    }
}
