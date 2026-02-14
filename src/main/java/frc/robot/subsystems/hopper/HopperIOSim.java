package frc.robot.subsystems.hopper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.HopperConstants;

public class HopperIOSim implements HopperIO {
        DCMotor m = DCMotor.getNeo550(1);
        private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
        private final SparkClosedLoopController pidController = hopperMotor.getClosedLoopController();
          private final SparkMaxSim simMotor = new SparkMaxSim(hopperMotor, m);
        private final SparkRelativeEncoderSim simEncoder = new SparkRelativeEncoderSim(hopperMotor);
      

    
//     DCMotor m = DCMotor.getNeo550(1);
//     private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
//     private final SparkMaxSim simMotor = new SparkMaxSim(hopperMotor, m);
//     private final PIDController pidController = new PIDController(HopperConstants.HOPPER_kP,HopperConstants.HOPPER_kI, HopperConstants.HOPPER_kD);
//     private final SparkRelativeEncoderSim simEncoder = simMotor.getRelativeEncoderSim();
//   //  private final LinearSystem<N2, N1, N2> lin = LinearSystemId.identifyPositionSystem(HopperConstants.HOPPER_kV, HopperConstants.HOPPER_kA);
//     private final LinearSystem<N2, N1, N2> lin = LinearSystemId.createDCMotorSystem(m, HopperConstants.HOPPER_kV, HopperConstants.HOPPER_kA);
//    // private final LinearSystemSim<N3, N1, N1> shit = new LinearSystemSim<N3, N1, N1>(null, null);
//    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<>(lin);
     private double hopperAppliedVolts = 0.0;
     public HopperIOSim() {
        SparkMaxConfig cfg = new SparkMaxConfig();

        cfg.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        cfg.closedLoop.maxMotion
            .cruiseVelocity(HopperConstants.HOPPER_CRUISE_VELOCITY)
            .maxAcceleration(HopperConstants.HOPPER_MAX_ACCELERATION)
            .allowedProfileError(HopperConstants.HOPPER_MAX_ALLOWED_PROFILER_ERROR);
        
        cfg.closedLoop.feedForward
            .kV(HopperConstants.HOPPER_kV)
            .kA(HopperConstants.HOPPER_kA)
            .kS(HopperConstants.HOPPER_kS);
        cfg.closedLoop
            .pid(HopperConstants.HOPPER_kP, HopperConstants.HOPPER_kI, HopperConstants.HOPPER_kD);

        hopperMotor.configure(cfg, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }
    
    /** Update the set of loggable inputs */
    @Override
    public void updateInputs(HopperIOInputs inputs) {
        
        simMotor.iterate(pidController.getMAXMotionSetpointVelocity(), 12, 0.02);
        
        inputs.hopperIAccum = pidController.getIAccum();
        
        inputs.hopperMotor_Volts =  simMotor.getBusVoltage();
        inputs.hopperMotor_Amps = simMotor.getMotorCurrent();
        inputs.hopperExtension_Rotations = simEncoder.getPosition();
        inputs.hopperExtension_Inches = inputs.hopperExtension_Rotations / HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION;
        inputs.hopperSetpoint_Inches = getGoal();
    }
   
    @Override
    public void goToPosition(double position_inches){
       pidController.setSetpoint(position_inches, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
    public boolean atSetpoint(){
        return pidController.isAtSetpoint();
   
    }
    public double getGoal(){
        return pidController.getSetpoint() / HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION;
    }
}
