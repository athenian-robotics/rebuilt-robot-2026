// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.firecontrol.ProjectileSimulator;
import frc.firecontrol.ShotCalculator;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.RuntimeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.BasicControlState;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.PathGeneration;
import frc.robot.util.AllianceUtil;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. 
 * Since Command-based is a "declarative" paradigm, very little robot logic 
 * should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls). 
 * Instead, the structure of the robot 
 * (including subsystems, commands, and button mappings) 
 * should be declared here.
 */
public class RobotContainer {
    // -- Subsystems --
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;
    private final Outtake outtake;
    private final Indexer indexer;

    private final PathGeneration pathGeneration;

    // -- Controllers --
    private final CommandJoystick driveJoystick = new CommandJoystick(ControllerConstants.JOYSTICK_LEFT_PORT);
    private final CommandJoystick steerJoystick = new CommandJoystick(ControllerConstants.JOYSTICK_MIDDLE_PORT);
    private final CommandJoystick operatorJoystick = new CommandJoystick(ControllerConstants.JOYSTICK_RIGHT_PORT);

    // -- Dashboard inputs --
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     * <ul>
     * <li> Manages REAL vs SIM vs REPLAY modes.
     * <li> Binds commands to joysticks and buttons.
     * <li> Creates and adds autos to dashboard.
     * </ul>
     */
    public RobotContainer() {
        switch (RuntimeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                vision  = new Vision(new VisionIOLimelight());
                drive   = new Drive(
                          vision,
                          new GyroIOPigeon2(),
                          new ModuleIOTalonFX(TunerConstants.FrontLeft),
                          new ModuleIOTalonFX(TunerConstants.FrontRight),
                          new ModuleIOTalonFX(TunerConstants.BackLeft),
                          new ModuleIOTalonFX(TunerConstants.BackRight));
                intake  = new Intake(new IntakeIOTalonFX());
                outtake = new Outtake(new OuttakeIOTalonFX());
                indexer = new Indexer(new IndexerIOTalonFX());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                vision  = new Vision(new VisionIO() {});
                drive   = new Drive(
                          vision,
                          new GyroIO() {},
                          new ModuleIOSim(TunerConstants.FrontLeft),
                          new ModuleIOSim(TunerConstants.FrontRight),
                          new ModuleIOSim(TunerConstants.BackLeft),
                          new ModuleIOSim(TunerConstants.BackRight));
                intake  = new Intake(new IntakeIOSim());
                outtake = null;
                indexer = new Indexer(new IndexerIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                vision  = new Vision(new VisionIO() {});
                drive   = new Drive(
                          vision,
                          new GyroIO() {},
                          new ModuleIO() {},
                          new ModuleIO() {},
                          new ModuleIO() {},
                          new ModuleIO() {});
                intake  = new Intake(new IntakeIO() {});
                outtake = new Outtake(new OuttakeIO() {});
                indexer = new Indexer(new IndexerIO() {});
        }

        // Create a new PathGeneration for autos
        pathGeneration = new PathGeneration();
        
        // Create basic named commands for autos
        NamedCommands.registerCommand("DeployHopperIntake",
                intake.runBasicControl(BasicControlState.FORWARD)
                .andThen( Commands.waitSeconds(4))
                .andThen( intake.runBasicControl(BasicControlState.STOPPED))
                .andThen( intake.runIntake()));

        NamedCommands.registerCommand("AimAndScore", 
                outtake.aimAtTarget(() -> drive.getPose().getTranslation())
                .andThen( outtake.startFlywheel())
                .andThen( Commands.waitUntil(outtake::isSpunUp))
                .andThen( indexer.toggle())
                .andThen( outtake.sendBallsToShooter())
                .andThen( Commands.waitSeconds(3)));

        NamedCommands.registerCommand("StartFlywheel", 
                outtake.startFlywheel());

        NamedCommands.registerCommand("LaunchFuel", 
                outtake.setAngle(() -> 40.0)
                .andThen( outtake.startFlywheel())
                .andThen( Commands.waitUntil(outtake::isSpunUp))
                .andThen( indexer.toggle())
                .andThen( outtake.sendBallsToShooter()));

        // Create dashboard item for choosing current auto
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", 
                DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", 
                DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", 
                drive.sysIdDynamicDrive(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", 
                drive.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Turn SysId (Quasistatic Forward)",
                drive.sysIdQuasistaticRotate(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Turn SysId (Quasistatic Reverse)",
                drive.sysIdQuasistaticRotate(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Turn SysId (Dynamic Forward)", 
                drive.sysIdDynamicRotate(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Turn SysId (Dynamic Reverse)", 
                drive.sysIdDynamicRotate(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Outtake SysId (Quasistatic Forward)",
                outtake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Outtake SysId (Quasistatic Reverse)",
                outtake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Outtake SysId (Dynamic Forward)", 
                outtake.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Outtake SysId (Dynamic Reverse)", 
                outtake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Intake SysId (Quasistatic Forward)",
                intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Intake SysId (Quasistatic Reverse)",
                intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Intake SysId (Dynamic Forward)", 
                intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Intake SysId (Dynamic Reverse)", 
                intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Speed At 12 Volts", drive.findMaxSpeed());

        // Configure the button bindings - assigns buttons and movement to joysticks
        configureJoystickBindings();

        PathfindingCommand.warmupCommand().schedule();

        // SOTM Lookup table
        // Measurements from CAD
        // TODO: REAL!
        ProjectileSimulator.SimParameters params = new ProjectileSimulator.SimParameters(
                0.215, // Ball mass (kg)
                0.1501, // Ball diameter (m)
                0.47, // Drag coeff (smooth sphere)
                0.2, // Magnus coeff
                1.225, // Air density
                0.43, // Exit height (m), floor to where the ball leaves the shooter
                0.1016, // Flywheel diameter (m), measure with calipers
                1.83, // Target height (m), from game manual
                0.6, // Slip factor (0=no grip, 1=perfect), tune this on the real robot
                45.0, // Launch angle from horizontal, measure from CAD
                0.001, // Sim timestep
                1500, 6000, 25, 5.0 // RPM search range, iterations, max sim time
        );

        ProjectileSimulator sim = new ProjectileSimulator(params);
        ProjectileSimulator.GeneratedLUT lut = sim.generateLUT();

        // Print the LUT for debugging
        for (var entry : lut.entries()) {
            if (entry.reachable()) {
                System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
                        entry.distanceM(), entry.rpm(), entry.tof());
            }
        }

        ShotCalculator.Config config = new ShotCalculator.Config();
        // TODO: REAL!
        config.launcherOffsetX = 0.23; // how far forward the launcher is from robot center (m)
        config.launcherOffsetY = 0.0; // how far left, 0 if centered
        config.phaseDelayMs = 30.0; // your vision pipeline latency
        config.mechLatencyMs = 20.0; // how long the mechanism takes to respond
        config.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps)
        config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
        config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub

        ShotCalculator shotCalc = new ShotCalculator(config);

        // load the LUT you generated
        for (var entry : lut.entries()) {
            if (entry.reachable()) {
                shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }

        // call this every cycle in robotPeriodic()
        Translation2d hubCenter = new Translation2d(4.6, 4.0); // your target
        Translation2d hubForward = new Translation2d(1, 0); // which way the hub faces

        // TODO: Uncomment these and implement getFieldVelocity and getRobotVelocity.
        
        // ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        //         drive.getPose(),
        //         drive.getFieldVelocity(),
        //         drive.getRobotVelocity(),
        //         hubCenter,
        //         hubForward,
        //         0.9, // vision confidence, 0 to 1
        //         drive.getPitch().getDegrees(), // pitch for tilt gate (0.0 if no gyro)
        //         drive.getRoll().getDegrees() // roll for tilt gate (0.0 if no gyro)
        // );

        // ShotCalculator.LaunchParameters shot = shotCalc.calculate(inputs);
        // if (shot.isValid() && shot.confidence() > 50) {
        //     outtake.setRPM(shot.rpm());
        //     drive.aimAt(shot.driveAngle());
        //     // shot.driveAngularVelocityRadPerSec() gives you a heading feedforward if you
        //     // want it
        // }

        // Operator can adjust trim
        // bind to copilot thumb-pad
        operatorJoystick.povRight().onTrue(Commands.runOnce(() -> shotCalc.adjustOffset(25)));
        operatorJoystick.povDown().onTrue(Commands.runOnce(() -> shotCalc.adjustOffset(-25)));
        // reset on mode change so trim doesn't carry over
        shotCalc.resetOffset();
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureJoystickBindings() {
    
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveJoystick.getY(),
            () -> -driveJoystick.getX(),
            () -> -steerJoystick.getX()));
  

    // This allows for heading-based drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDriveAtAngle(
    //         drive,
    //         () -> driveJoystick.getY(),
    //         () -> driveJoystick.getX(),
    //         () -> new Rotation2d(-steerJoystick.getY(), -steerJoystick.getX()),
    //         () -> steerJoystick.getMagnitude() >=
    // Constants.ControllerConstants.HEADING_DEADZONE));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));


    operatorJoystick.button(ControllerConstants.THUMB_BUTTON_BOTTOM).toggleOnTrue(outtake.groundOuttake().alongWith(indexer.hold()));

    /**
    operatorJoystick.button(ControllerConstants.THUMB_BUTTON_RIGHT).onTrue(
      intake.runIntake()
    );
    */
    steerJoystick
        .button(ControllerConstants.THUMB_BUTTON_BOTTOM)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Rotation2d desiredHeading =
                          AllianceUtil.isRedAlliance()
                              ? new Rotation2d(Degrees.of(180))
                              : new Rotation2d();
                      drive.setDriverFieldRelativeHeading(desiredHeading);
                    },
                    drive)
                .ignoringDisable(true));

        // Heading-based drive
        // drive.setDefaultCommand(
        // DriveCommands.joystickDriveAtAngle(
        // drive,
        // () -> driveJoystick.getY(),
        // () -> driveJoystick.getX(),
        // () -> new Rotation2d(-steerJoystick.getY(), -steerJoystick.getX()),
        // () -> steerJoystick.getMagnitude() >=
        // Constants.ControllerConstants.HEADING_DEADZONE));

        // Lock angle to 0° while button "A" is held
        // controller
        // .a()
        // .whileTrue(
        // DriveCommands.joystickDriveAtAngle(
        // drive,
        // () -> -controller.getLeftY(),
        // () -> -controller.getLeftX(),
        // () -> new Rotation2d()));

        // Switch to X pattern (braking) when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Run intake
        // operatorJoystick.button(ControllerConstants.THUMB_BUTTON_RIGHT).onTrue(
        // intake.runIntake()
        // );

        // Should return the bot to its initial position
        // driveJoystick
        // .button(ControllerConstants.TRIGGER)
        // .whileTrue(
        // Commands.runOnce(() -> System.out.println("Running path gen"))
        // .andThen(pathGeneration.pathfindTo(Location.TEST_POSE)));

        // Might work better
        // driveJoystick
        // .button(ControllerConstants.THUMB_BUTTON_RIGHT)
        // .onTrue(pathGeneration.pathfindToSimple(drive::getPose, Location.TEST_POSE,
        // 0.0));

        // Aim shooter with operator joystick when holding left side top right button
        // operatorJoystick.button(ControllerConstants.OFFHAND_TOP_RIGHT).whileTrue(outtake.aimWithJoystick(()
        // -> operatorJoystick.getY()));
        // Aim shooter from value in dashboard when holding left side top middle button
        // operatorJoystick.button(ControllerConstants.OFFHAND_TOP_MIDDLE).onTrue(outtake.toNTAngle().andThen(outtake.updateDistance(()
        // -> drive.getPose().getTranslation(), () ->
        // OuttakeConstants.HUB_POSITION_BLUE)));

        driveJoystick.button(ControllerConstants.THUMB_BUTTON_BOTTOM).whileTrue(outtake.keepAimingAtTarget(() -> drive.getPose().getTranslation()));

        driveJoystick
        .button(ControllerConstants.TRIGGER)
        .whileTrue(
            outtake
                .aimAtTarget(() -> drive.getPose().getTranslation())
                .andThen(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driveJoystick.getY(),
                        () -> -driveJoystick.getX(),
                        () -> {
                            var hubPosition =
                                AllianceUtil.isRedAlliance()
                                    ? OuttakeConstants.HUB_POSITION_RED
                                    : OuttakeConstants.HUB_POSITION_BLUE;
                            return hubPosition.minus(drive.getPose().getTranslation()).getAngle();
                        })));
        
        // Operator left side bottom left button    -->  lower hood
        operatorJoystick.button(ControllerConstants.OFFHAND_BOTTOM_LEFT).onTrue(outtake.lowerHood());
        // Operator left side bottom middle button  -->  send balls to shooter (with indexer rollers)
        operatorJoystick.button(ControllerConstants.OFFHAND_BOTTOM_MIDDLE).whileTrue(outtake.sendBallsToShooter().alongWith(indexer.hold()));
        // Operator left side bottom middle button  -->  send balls to shooter
        operatorJoystick.button(ControllerConstants.OFFHAND_BOTTOM_RIGHT).onTrue(outtake.aimAtTarget(() -> drive.getPose().getTranslation()));
        // Operator left side top right button      -->  snap shooter angle to 40º
        operatorJoystick.button(ControllerConstants.OFFHAND_TOP_RIGHT).onTrue(outtake.setAngle(() -> 40));

        // Steer trigger -> send balls to shooter (without indexer rollers)
        steerJoystick.button(ControllerConstants.TRIGGER).whileTrue(outtake.sendBallsToShooter());

        // Operator right thumb                     -> start flywheel
        operatorJoystick.button(ControllerConstants.THUMB_BUTTON_RIGHT).onTrue(outtake.startFlywheel());
        // Operator left thumb                      -> stop flywheel
        operatorJoystick.button(ControllerConstants.THUMB_BUTTON_LEFT).onTrue(outtake.stopFlywheel());
        // Operator right thumb                     -> toggle intake
        operatorJoystick.button(ControllerConstants.TRIGGER).toggleOnTrue(intake.runIntake());
        // Operator right side top right            -> open hopper
        // operatorJoystick.button(ControllerConstants.MAINHAND_TOP_RIGHT).whileTrue(intake.openHopper());
        // Operator right side top middle           -> close hopper and intake
        // operatorJoystick.button(ControllerConstants.MAINHAND_TOP_MIDDLE).whileTrue(intake.fullyRetract());
        operatorJoystick.button(ControllerConstants.MAINHAND_TOP_LEFT).onTrue(intake.runBasicControl(BasicControlState.FORWARD));
        operatorJoystick.button(ControllerConstants.MAINHAND_TOP_MIDDLE).onTrue(intake.runBasicControl(BasicControlState.STOPPED));
        operatorJoystick.button(ControllerConstants.MAINHAND_TOP_RIGHT).onTrue(intake.runBasicControl(BasicControlState.BACKWARD));
        // Operator right side bottom left          -> lower hood
        operatorJoystick.button(ControllerConstants.MAINHAND_BOTTOM_LEFT).onTrue(outtake.setAngle(() -> OuttakeConstants.LOW_SET_ANGLE_DEG));
        // Operator right side bottom middle        -> set hood to middle
        operatorJoystick.button(ControllerConstants.MAINHAND_BOTTOM_MIDDLE).onTrue(outtake.setAngle(() -> OuttakeConstants.MIDDLE_SET_ANGLE_DEG));
        // Operator right side bottom right         -> lift hood
        operatorJoystick.button(ControllerConstants.MAINHAND_BOTTOM_RIGHT).onTrue(outtake.setAngle(() -> OuttakeConstants.HIGH_SET_ANGLE_DEG));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
