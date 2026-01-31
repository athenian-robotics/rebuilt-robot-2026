package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class Indexer extends SubsystemBase {
    private final TalonFX motor; 
    
    public Indexer() {
        motor = new TalonFX(IndexerConstants.MOTOR_ID);
    }

    public Command setIndexerSpeed(double speed) {
        Logger.recordOutput("Indexer/speed", speed);
        return new InstantCommand(() -> motor.set(speed));
    }
}
