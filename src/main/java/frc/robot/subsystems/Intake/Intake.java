import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void fullyExtend(){ //position feedback loop

  }
  public void wiggleUp(){//velocity 

  }
  public void wiggleDown(){

  }
  public void wiggle(){
    wiggleUp();
    wiggleDown();
  }
}
