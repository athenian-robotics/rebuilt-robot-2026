import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX armMotor = new TalonFX(IntakeConstants.TALON_ID);

  private final TalonFXConfigurator configuration = armMotor.getConfigurator();

  /** Update the set of loggable inputs */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armMotor_Voltage_Volts = (armMotor.getMotorVoltage().getValueAsDouble());
    inputs.armMotor_Current_Amps = armMotor.getTorqueCurrent().getValueAsDouble();
    inputs.armMotorRotations_Rotations = armMotor.getPosition().getValueAsDouble();
    inputs.armRotations_Rotations =
        inputs.armMotorRotations_Rotations * IntakeConstants.GEAR_ROTATIONS_TO_ARM_ROTATIONS;
  }

  @Override
  public void goToPosition(double rotations) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'goToPosition'");
  }
}
