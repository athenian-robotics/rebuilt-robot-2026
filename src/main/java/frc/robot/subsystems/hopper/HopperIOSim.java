package frc.robot.subsystems.hopper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.HopperConstants;

public class HopperIOSim implements HopperIO {
   
    
    DCMotor m = DCMotor.getNeo550(1);

    
    private final SparkMax hopperMotor = new SparkMax(HopperConstants.SPARK_ID, MotorType.kBrushless);
    private final SparkMaxSim simMotor = new SparkMaxSim(hopperMotor, m);
    private final PIDController pidController = new PIDController(HopperConstants.HOPPER_kP,HopperConstants.HOPPER_kI, HopperConstants.HOPPER_kD);
    private final SparkRelativeEncoderSim simEncoder = simMotor.getRelativeEncoderSim();
   // private final LinearSystemSim<N3, N1, N1> shit = new LinearSystemSim<N3, N1, N1>(null, null);
   private final LinearSystemSim<N3, N1, N1> shit = new LinearSystemSim<N3, N1, N1>(null, null);
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

        simMotor.iterate(2, 12, .02);        
        
        inputs.hopperMotor_Volts =  hopperAppliedVolts;
        inputs.hopperMotor_Amps = simMotor.getMotorCurrent();
        inputs.hopperExtension_Rotations = simEncoder.getPosition();
        inputs.hopperExtension_Inches = inputs.hopperExtension_Rotations / HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION;
    }
   
    @Override
    public void goToPosition(double position_inches){
        hopperAppliedVolts = pidController.calculate(simEncoder.getPosition(), position_inches * HopperConstants.HOPPER_POSITION_TO_ANGLE_CONVERSION);
        simMotor.setBusVoltage(hopperAppliedVolts);
    }
    public boolean atSetpoint(){
        return pidController.atSetpoint();
   
    }
    public double getGoal(){
        return pidController.getSetpoint();
    }
}
