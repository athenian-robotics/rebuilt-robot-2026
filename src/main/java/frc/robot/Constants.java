package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

public class Constants {
  public static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(70)
    .withSupplyCurrentLowerLimit(37)
    .withSupplyCurrentLowerTime(1)
    .withSupplyCurrentLimitEnable(true);

  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
  public class RuntimeConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }
  }

  public final class ControllerConstants {
    // -- Port (identifier) for each individual joystick/controller.
    public static final int JOYSTICK_LEFT_PORT = 0;
    public static final int JOYSTICK_MIDDLE_PORT = 1;
    public static final int JOYSTICK_RIGHT_PORT = 2;

    /* -- Individual mappings for buttons on each joystick, from 1-16. Note that right-handed
     * -- controllers and left-handed controllers have opposite and reflected bindings (see below).
     */
    public static final int TRIGGER = 1;
    public static final int THUMB_BUTTON_BOTTOM = 2;
    public static final int THUMB_BUTTON_LEFT = 3;
    public static final int THUMB_BUTTON_RIGHT = 4;

    /* Each joystick is either left or right handed. When holding the joystick, the buttons on
     * the side of the opposing hand are the offhand buttons. The buttons on the side of the hand
     * holding the joystick is the mainhand buttons.
     */
    public static final int OFFHAND_TOP_LEFT = 5;
    public static final int OFFHAND_TOP_MIDDLE = 6;
    public static final int OFFHAND_TOP_RIGHT = 7;
    public static final int OFFHAND_BOTTOM_RIGHT = 8;
    public static final int OFFHAND_BOTTOM_MIDDLE = 9;
    public static final int OFFHAND_BOTTOM_LEFT = 10;

    public static final int MAINHAND_TOP_RIGHT = 11;
    public static final int MAINHAND_TOP_MIDDLE = 12;
    public static final int MAINHAND_TOP_LEFT = 13;
    public static final int MAINHAND_BOTTOM_LEFT = 14;
    public static final int MAINHAND_BOTTOM_MIDDLE = 15;
    public static final int MAINHAND_BOTTOM_RIGHT = 16;
  }

  public final class DrivetrainConstants {
    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;
  }

  public static final class LimelightConstants {
    /** NetworkTables name configured on the camera. */
    public static final String CAMERA_NAME = "limelight";

    /**
     * Transform from the robot origin (center of rotation on the floor) to the camera pose.
     */
    public static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(Inches.of(0.0), Inches.of(0.0), Inches.of(7.5)),
            new Rotation3d(0.0, Units.degreesToRadians(28.1), 0.0));
    // TODO: update to the final measured translation/rotation from robot origin to camera.

    /** Maximum pose ambiguity reported by Limelight to accept a measurement. */
    public static final double MAX_POSE_AMBIGUITY =
        0.2; // Lower value rejects noisy solves, raise if too many drops occur.
    /** Minimum number of tags contributing to the solve to accept a pose. */
    public static final int MIN_TAG_COUNT = 1; // Set to 2+ if single-tag jitter is problematic.

    /**
     * Baseline standard deviations (meters) for measurements that only use a single tag. Multi-tag
     * solves will scale these values down dynamically.
     */
    public static final double SINGLE_TAG_XY_STDDEV =
        0.45; // Reduce if the camera is steady on good tags.

    public static final double SINGLE_TAG_THETA_STDDEV =
        Units.degreesToRadians(7.5); // Adjust if heading noise is smaller/bigger.
    /** Floor for the best possible standard deviation. */
    public static final double MIN_XY_STDDEV = 0.05; // Safety floor for downstream pose estimators.

    public static final double MIN_THETA_STDDEV =
        Units.degreesToRadians(2.0); // Prevent unrealistically low heading noise.

    /** Distance where we start to inflate std devs significantly (meters). */
    public static final double DISTANCE_TRUST_FALLOFF_METERS =
        4.5; // Tune to how vision accuracy drops off with range.

    /** Maximum allowed pose translation error between vision and odometry before rejecting data. */
    public static final double MAX_TRANSLATION_ERROR_METERS = 2.0;

    /**
     * Maximum allowed rotation error (radians) between vision and odometry before rejecting data.
     */
    public static final double MAX_ROTATION_ERROR_RADIANS = Units.degreesToRadians(30.0);

    /**
     * Minimum translation correction (meters) before a vision update is fused into odometry.
     * Smaller corrections are treated as jitter and ignored.
     */
    public static final double MIN_VISION_CORRECTION_TRANSLATION_METERS = 0.03;

    /**
     * Minimum rotation correction (radians) before a vision update is fused into odometry.
     * Smaller corrections are treated as jitter and ignored.
     */
    public static final double MIN_VISION_CORRECTION_ROTATION_RADIANS =
        Units.degreesToRadians(1.5);

    /** The maximum time before an observation is no longer considered fresh (seconds). */
    public static final double FRESH_OBSERVATION_THRESHOLD = 0.5;

    /** The maximum number of times that vision can be overridden by odometry in a row. Set to Integer.MAX_VALUE to disable behavior. */
    public static final int MAX_SEQUENTIAL_REJECTIONS = 60; 

    /** 
     * The number of times in a row to force override odometry when overriding odometry.
     * Necessary because otherwise the pose estimate will move some towards the vision measurement, 
     * but then vision will be rejected again. 
     */
    public static final int SEQUENTIAL_OVERRIDES = 5;
  }

  public final class DriveCommandsConstants {
    public static final double DEADBAND = 0.1;
    public static final double ANGLE_KP = 8.0;
    public static final double ANGLE_KD = 0;
    public static final double ANGLE_MAX_VELOCITY = 16.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  }

  public final class OuttakeConstants {
    // motor ids
    public static final int RIGHT_SHOOTER_MOTOR = 27;
    public static final int LEFT_SHOOTER_MOTOR = 22;
    public static final int MIDDLE_WHEEL_MOTOR = 24;
    public static final int STAR_WHEEL_MOTOR = 23;
    public static final int ANGLE_CHANGER_MOTOR = 50;

    public static final double MIDDLE_WHEEL_TO_SHOOTER_VOLTS = 10;
    public static final double MIDDLE_WHEEL_TO_GROUND_VOLTS = -3;
    public static final double STAR_WHEEL_TO_GROUND_VOLTS = 7;
    public static final double STAR_WHEEL_TO_SHOOTER_VOLTS = 10;
    public static final double FLYWHEEL_VOLTS = 12;
    
    public static final double FLYWHEEL_MAX_ERROR_RPS = 10.0;
    public static final double FLYWHEEL_VELOCITY_RPS = 80.0;

    public static final double MAXIMUM_HOOD_ANGLE_DEG = 41.217118;
    public static final double MINIMUM_HOOD_ANGLE_DEG = 15.447513;
    public static final double STARTING_HOOD_ANGLE_DEG = 15.447513;
    public static final double MIDFIELD_SHOT_ANGLE_DEG = 35; // guess
    public static final double OPPOSITE_TEAM_SHOT_ANGLE_DEG = 40; //guess

    public static final double MIDFIELD_LIMIT_FEET = 15.18; // from allied team driverstation wall
    public static final double OPPOSITE_TEAM_LIMIT_FEET = 39.092; // from allied team driverstation wall

    public static final double ANGLE_CHANGER_GEAR_RATIO = 1.0 / (12.0 * 16.0);

    public static final double HOOD_ANGLE_KP = 35; //35.203
    public static final double HOOD_ANGLE_KD = 0; //8.0464
    public static final double HOOD_ANGLE_KS = 0.017;
    public static final double HOOD_ANGLE_KV = 1.7; //19.949
    public static final double HOOD_ANGLE_KA = 0; //2.0061

    public static final double HOOD_ANGLE_CRUISE_VELOCITY_RPS = 0.05;
    public static final double HOOD_ANGLE_MAX_ACCELERATION_RPSPS = 0.1;

    public static final double ANGLE_CHANGER_STARTING_ANGLE_ROTATIONS = STARTING_HOOD_ANGLE_DEG / 360.0;

    public static final Translation2d HUB_POSITION_BLUE = new Translation2d(Inches.of(182.1), Inches.of(158.85)); // x for facing 126.98 inches y for facing -38.00039 inches
    public static final Translation2d HUB_POSITION_RED = new Translation2d(Inches.of(469.1), Inches.of(158.85));

    public static final double LOW_SET_ANGLE_DEG = 23.2926886432; // 2.4333333333 m
    public static final double MIDDLE_SET_ANGLE_DEG = 28.2261055557; // 3.3166666667 m
    public static final double HIGH_SET_ANGLE_DEG = 30.0072; // 4.2 m
  }

  public final class IntakeConstants {
    // motors
    public static final int ARM_ID = 26;
    public static final int WHEEL_ID = 25;
    public static final double GEAR_ROTATIONS_TO_ARM_ROTATIONS =
        1.0 / 180.0;

    // feedback constants
    // public static final double INTAKE_kP = 5.5824; // 55.824
    // public static final double INTAKE_kI = 0;
    // public static final double INTAKE_kD = .37177; //3.7177
    // feedforward constants
    // public static final double INTAKE_kS = 0.41289;
    // public static final double INTAKE_kV = 20.206;
    // public static final double INTAKE_kA = 0.90771;
    // public static final double INTAKE_kG = 0.36108;

    // motion profiler constants
    // public static final double INTAKE_CRUISE_VELOCITY = 12;
    // public static final double INTAKE_MAX_ACCELERATION = 100;
    // public static final double INTAKE_MAX_ALLOWED_PROFILER_ERROR = 5.0/360.0;

    public static final double BASIC_CONTROL_FORWARD_VOLTS = 5.0;
    public static final double BASIC_CONTROL_BACKWARD_VOLTS = 5.0;
    // public static final double BASIC_CONTROL_TOLERANCE_DEG = 5.0/360.0;

    public static final double MAX_ARM_VOLTAGE = 7;
    public static final double WHEEL_VOLTAGE = 6;
    public static final double ARM_STARTING_POSITION_ROT = 1.0 / 3.0;
  }

  public final class IndexerConstants {
    public static final int MOTOR_ID = 52;
    public static final double MOTOR_VOLTAGE = 8.0;
  }

  public final class PathGenerationConstants {

    // Predefined locations of interest
    public enum Location {
      TEST_POSE(new Pose2d(14.55, 1.0, new Rotation2d()));

      private final Pose2d pose;

      Location(Pose2d pose) {
        this.pose = pose;
      }

      public Pose2d getPose() {
        return pose;
      }
    }

    // Default constraints for pathfinding
    // Adjust these based on your robot's capabilities
    public static final double MAX_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ACCELERATION = 2.8; //from pathplanner
    public static final double MAX_ANGULAR_VELOCITY =
        MAX_VELOCITY / Math.sqrt(Math.pow(0.273, 2) * 2);
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(790);

    public static final PathConstraints DEFAULT_CONSTRAINTS =
        new PathConstraints(
            MAX_VELOCITY, // Max velocity (m/s)
            MAX_ACCELERATION, // Max acceleration (m/s^2) (from PathPlanner)
            MAX_ANGULAR_VELOCITY, // Max angular velocity (rad/s)
            MAX_ANGULAR_ACCELERATION); // Max angular acceleration (rad/s^2) (from PathPlanner)
  }

  public final class CANConstants {
    public static final String CANIVORE_NAME = "can"; // 7733663E3353385320202034382203FF
  }

  public final class SOTMConstants {
    public static final double BALL_MASS_KG = 0.215; // From original code
    public static final double BALL_DIAMETER_M = 0.1501; // From original code
    public static final double BALL_DRAG_COEFFICIENT = 0.47; // From original code
    public static final double BALL_MAGNUS_COEFFICIENT = 0.2; // From original code
    public static final double AIR_DENSITY = 1.225; // From original code
    public static final double EXIT_HEIGHT_M = 0.5588; // Exit height (m), floor to where the ball leaves the shooter
    public static final double WHEELE_DIAMETER_M = 0.1016; // Flywheel diameter (m), measure with calipers
    public static final double TARGET_HEIGHT_M = 1.83; // Hub height from game manual
    public static final double SLIP_FACTOR = 0.6; // Slip factor (0=no grip, 1=perfect), tune this on the real robot

    public static final double LAUNCHER_OFFSET_X_M = -0.16764;
    public static final double LAUNCHER_OFFSET_Y_M = 0.0;
    public static final double PHASE_DELAY_MS = 30.0;
    public static final double MECH_LATENCY_MS = 20.0;
    public static final double MAX_TILT_DEG = 5.0;
    public static final double HEADING_SPEED_SCALAR_MPS = 1.0;
    public static final double HEADING_REFERENCE_DISTANCE_M = 2.5;
    
    public static final double MIN_SCORING_DISTANCE_M = 0.0;
    public static final double MAX_SCORING_DISTANCE_M = 0.0;
    public static final int MAX_ITERATIONS = 0;
    public static final double CONVERGENCE_TOLERANCE = 0.001;
    public static final double TOF_MIN = 0.05;
    public static final double TOF_MAX = 5.0;
    public static final double SOTM_DRAG_COEFF = 0.47;
    public static final double MIN_SOTM_SPEED = 0.1; // Below this speed (m/s), don't bother with SOTM, just aim straight
    public static final double MAX_SOTM_SPEED = 3.0; // Above this speed (m/s), don't shoot, we're outside calibration range

    // Confidence scoring weights (5-component weighted geometric mean)
    public static final double W_CONVERGENCE = 1.0;
    public static final double W_VELOCITY_STABILITY = 0.8;
    public static final double W_VISION_CONFIDENCE = 1.2;
    public static final double W_HEADING_ACCURACY = 1.5;
    public static final double W_DISTANCE_IN_RANGE = 0.5;
    public static final double HEADING_MAX_ERROR_RAD = Math.toRadians(15);

    public static final double HEIGHT_TOLERANCE_M = 0.02;
    public static final double TRIM_MAX = 20;
    

  }
}
