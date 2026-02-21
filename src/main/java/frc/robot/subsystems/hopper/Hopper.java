package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.HopperConstants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private HopperIO io;
    private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public Hopper(HopperIO io){
        this.io = io;
        
        Config sysIdConfig = new Config(Volts.per(Seconds).of(.5), Volts.of(3), Seconds.of(5),
            (state) -> Logger.recordOutput("Hopper/SysIdState", state.toString()));
        Mechanism sysIdMechanism = new Mechanism((volts) -> io.runSysid(volts.in(Volts)), null, this);

        sysId = new SysIdRoutine(sysIdConfig, sysIdMechanism);
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
        io.periodic();
    }

    /**
     * Retracts the hopper to fully retracted
     */
    public void retract() {
        io.goToPosition(HopperConstants.HOPPER_RETRACTED * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
    }
    

    /**
     * Moves the hopper to {@value HopperConstants#HOPPER_PARTIAL} inches
     */
    public void partial() {
        io.goToPosition(HopperConstants.HOPPER_PARTIAL * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
        //io.goToPosition();
    }
    /**
     * Moves the hopper to max extension of {@value HopperConstants#HOPPER_FULL}
     */
    public void full() {
        io.goToPosition(HopperConstants.HOPPER_FULL * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
    }
    public boolean atSetpoint(){
        return io.atSetpoint();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.runSysid(0.0))
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
        }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.runSysid(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

}
