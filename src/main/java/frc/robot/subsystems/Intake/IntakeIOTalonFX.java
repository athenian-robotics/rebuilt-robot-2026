import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intakeArmMotor = new TalonFX(HopperConstants.SPARK_ID, MotorType.kBrushless);
    
    
    
    /** Update the set of loggable inputs */
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeArmMotor_Volts =  intakeMotor.getBusVoltage();
        inputs.intakeArmMotor_Amps = intakeMotor.getOutputCurrent();
        
    }
   
   
}