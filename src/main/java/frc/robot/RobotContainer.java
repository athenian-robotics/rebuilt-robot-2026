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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
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
import frc.robot.subsystems.outtake.OuttakeIOSim;
import frc.robot.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceUtil;
import frc.robot.util.PathGeneration;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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
                outtake = new Outtake(new OuttakeIOSim());
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
                intake.setAngle(IntakeConstants.ARM_ENDING_POSITION_ROT)
                .andThen(Commands.waitSeconds(3))
                .andThen(intake.wiggleTo(-70).withTimeout(1))
                .andThen(Commands.waitSeconds(2))
                .andThen(intake.wiggleTo(-70).withTimeout(1))
                .andThen(Commands.waitSeconds(2))
                .andThen(intake.runIntake()));

        NamedCommands.registerCommand("AimAndScore", 
                Commands.print("aiming")
                .andThen(outtake.aimAtTarget(() -> drive.getPose().getTranslation()))
                .andThen( Commands.print("flywheeling"))
                .andThen( outtake.startFlywheel())
                .andThen( Commands.waitUntil(outtake::isSpunUp))
                .andThen( indexer.hold())
                .alongWith( outtake.sendBallsToShooter())
                .withTimeout(8)
                .andThen( intake.stopIntake()));

        NamedCommands.registerCommand("StartFlywheel", 
                outtake.startFlywheel());

        NamedCommands.registerCommand("LaunchFuel", 
                outtake.setAngle(() -> 40.0)
                .andThen( outtake.startFlywheel())
                .andThen( Commands.waitUntil(outtake::isSpunUp))
                .andThen( indexer.hold())
                .alongWith( outtake.sendBallsToShooter())
                .withTimeout(8)
                .andThen( intake.stopIntake()));

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

        if (AllianceUtil.isRedAlliance()) {
            drive.setPose(new Pose2d(0, 0, new Rotation2d(180)));
        } else {
            drive.setPose(new Pose2d(0, 0, new Rotation2d(0)));
        }

        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Use this method to define your button -> command mappings. 
     * <p> Buttons can be created by instantiating a {@link GenericHID} 
     * or one of its subclasses: ({@link edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), 
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureJoystickBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driveJoystick.getY(),
                        () -> -driveJoystick.getX(),
                        () -> -steerJoystick.getX()));

        // Run indexer rollers and ground outtake balls
        operatorJoystick.button(ControllerConstants.THUMB_BUTTON_BOTTOM)
        .toggleOnTrue(outtake.groundOuttake().alongWith(indexer.hold()));

        driveJoystick.button((ControllerConstants.THUMB_BUTTON_BOTTOM)).onTrue(DriveCommands.brake(drive));
        
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

        steerJoystick
        .button(ControllerConstants.THUMB_BUTTON_LEFT)
        .onTrue(
            Commands.runOnce(
                    () -> {
                        Pose2d desiredPose =
                          AllianceUtil.isRedAlliance()
                              ? new Pose2d(Meters.of(13.03), Meters.of(4.035), new Rotation2d(Degrees.of(180)))
                              : new Pose2d(Meters.of(3.510), Meters.of(4.035), new Rotation2d());
                        drive.setPose(desiredPose);
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

        
        // Operator left side bottom left button    -->  lower hood
        operatorJoystick.button(ControllerConstants.OFFHAND_BOTTOM_LEFT).onTrue(outtake.lowerHood());
        // Operator left side bottom middle button  -->  send balls to shooter (with indexer rollers)
        operatorJoystick.button(ControllerConstants.TRIGGER).whileTrue(outtake.sendBallsToShooter().alongWith(indexer.hold()));
        // Operator left side bottom middle button  -->  send balls to shooter
        operatorJoystick.button(ControllerConstants.OFFHAND_BOTTOM_RIGHT).onTrue(outtake.aimAtTarget(() -> drive.getPose().getTranslation()));
        // Operator left side top right button      -->  snap shooter angle to 40º
        operatorJoystick.button(ControllerConstants.MAINHAND_TOP_RIGHT).onTrue(outtake.setAngle(() -> 40));

        // Steer trigger -> send balls to shooter (without indexer rollers)
        steerJoystick.button(ControllerConstants.TRIGGER).whileTrue(outtake.sendBallsToShooter());

        // Operator right thumb                     -> start flywheel
        operatorJoystick.button(ControllerConstants.THUMB_BUTTON_RIGHT).onTrue(outtake.startFlywheel());
        // Operator left thumb                      -> stop flywheel
        operatorJoystick.button(ControllerConstants.THUMB_BUTTON_LEFT).onTrue(outtake.stopFlywheel());
        // Operator right thumb                     -> toggle intake
        operatorJoystick.button(ControllerConstants.OFFHAND_BOTTOM_MIDDLE).toggleOnTrue(intake.runIntake());
        // Operator right side top right            -> open hopper
        // operatorJoystick.button(ControllerConstants.MAINHAND_TOP_RIGHT).whileTrue(intake.openHopper());
        // Operator right side top middle           -> close hopper and intake
        // operatorJoystick.button(ControllerConstants.MAINHAND_TOP_MIDDLE).whileTrue(intake.fullyRetract());
        operatorJoystick.button(ControllerConstants.OFFHAND_TOP_MIDDLE).whileTrue(intake.wiggleTo(-80));
        operatorJoystick.button(ControllerConstants.OFFHAND_TOP_RIGHT).onTrue(intake.setAngle(IntakeConstants.ARM_ENDING_POSITION_ROT * 360.0));
        // Operator right side bottom left          -> lower hood
        operatorJoystick.button(ControllerConstants.MAINHAND_BOTTOM_LEFT).onTrue(outtake.setAngle(() -> OuttakeConstants.LOW_SET_ANGLE_DEG));
        // Operator right side bottom middle        -> set hood to middle
        operatorJoystick.button(ControllerConstants.MAINHAND_BOTTOM_MIDDLE).onTrue(outtake.setAngle(() -> OuttakeConstants.MIDDLE_SET_ANGLE_DEG));
        // Operator right side bottom right         -> lift hood
        operatorJoystick.button(ControllerConstants.MAINHAND_BOTTOM_RIGHT).onTrue(outtake.setAngle(() -> OuttakeConstants.HIGH_SET_ANGLE_DEG));
        
        // Operator thumb pad -> trim hood angle
        operatorJoystick.povUp().onTrue(outtake.addTrim(OuttakeConstants.HOOD_ANGLE_TRIM_AMOUNT_DEGREES));
        operatorJoystick.povDown().onTrue(outtake.addTrim(-OuttakeConstants.HOOD_ANGLE_TRIM_AMOUNT_DEGREES));
        // Operator left side top left -> reset hood angle trim
        operatorJoystick.button(ControllerConstants.OFFHAND_TOP_LEFT).onTrue(outtake.resetTrim());
    
        // operatorJoystick.button(ControllerConstants.OFFHAND_TOP_RIGHT).whileTrue(outtake.aimWithJoystick(() -> operatorJoystick.getY()));
        // operatorJoystick.button(ControllerConstants.OFFHAND_TOP_MIDDLE).onTrue(outtake.toNTAngle().andThen(outtake.updateDistance(() -> drive.getPose().getTranslation(), () -> OuttakeConstants.HUB_POSITION_BLUE)));
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

        driveJoystick
            .button((ControllerConstants.THUMB_BUTTON_BOTTOM))
            .whileTrue(DriveCommands.brake(drive));
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
