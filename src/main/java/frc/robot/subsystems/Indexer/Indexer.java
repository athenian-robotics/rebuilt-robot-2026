package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;
    
    public Indexer() {
        io = new IndexerIOTalonFX();
        inputs = new IndexerIOInputsAutoLogged();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    /**
     * Run the indexer with a specified voltage.
     * @param voltage The voltage at which to run the indexer
     * @return The command
     */
    public Command runIndexer(Voltage voltage) {
        return new InstantCommand(() -> io.setVoltage(voltage), this);
    }
}
