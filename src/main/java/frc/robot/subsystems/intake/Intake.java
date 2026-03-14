package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.intake.IntakeIOTalonFX.BasicControlState;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private SysIdRoutine sysId;

    /**
     * Contains code to run intake wheels, move intake arm, and manage SysId autos.
     * <p> Also opens hopper using intake.
     * @param io The io object to use
     */
    public Intake(IntakeIO io) {
        this.io = io;

        Config sysIdConfig = new Config(
                Volts.per(Seconds).of(1), 
                Volts.of(3), 
                Seconds.of(5)
        );
        Mechanism sysIdMechanism = new Mechanism(
                (volts) -> io.runSysId(volts.in(Volts)), 
                io::sysIDLog, 
                this
        );

        sysId = new SysIdRoutine(
                sysIdConfig, 
                sysIdMechanism
        );
    }

    @Override
    public void periodic() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command runBasicControl(BasicControlState direction) {
        return Commands.runOnce(() -> io.goWithBasicControl(direction), this);
    }

    public Command runIntake() {
        return Commands.startEnd(io::startIntake, io::stopIntake, this);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.runSysId(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.runSysId(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }
}
