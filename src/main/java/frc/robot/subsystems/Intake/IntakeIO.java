import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double intake_Voltage = 0;
        public double intake_Current = 0;
    }
    public void updateInputs(IntakeIOInputs inputs);
    public double gearRotationsToArmRotations(double rotations);
    public void goToPosition(double rotations);
}
