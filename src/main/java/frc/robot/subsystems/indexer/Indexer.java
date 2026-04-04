package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;
    private State state = State.OFF;

    private enum State {
        ON,
        OFF
    }
    
    public Indexer(IndexerIO indexerIO) {
        io = indexerIO;
        inputs = new IndexerIOInputsAutoLogged();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    /**
     * Toggles the indexer between on and off
     * @return An instant command to do this
     */
    public Command toggle () {
        return Commands.either(
            Commands.runOnce(() -> {
                io.setVoltage(0);
                state = State.OFF; // If on toggle off
            }), 
            Commands.runOnce(() -> {
                io.setVoltage(-IndexerConstants.MOTOR_VOLTAGE);
                state = State.ON; // if off toggle on
            }), 
            () -> this.state == State.ON // Do top command if it's currently on
        );
    }

    /**
     * Runs the indexer as long as the command is running
     * @return A continuous command to do this
     */
    public Command hold () {
        return Commands.startEnd(() -> io.setVoltage(-IndexerConstants.MOTOR_VOLTAGE), () -> io.setVoltage(0), this);
    }
}
