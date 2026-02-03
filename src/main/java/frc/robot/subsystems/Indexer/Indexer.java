package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;
    
    public Indexer(IndexerIO indexerIO) {
        io = indexerIO;
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

    public IndexerIO getIndexerIO() {return io;}
}
