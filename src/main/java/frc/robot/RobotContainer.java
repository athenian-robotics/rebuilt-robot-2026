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
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PathGenerationConstants.Location;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.RuntimeConstants;
import frc.robot.commands.DriveCommands;  
import frc.robot.commands.HopperIntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.PathGeneration;

import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // -- Subsystems --
  private final Drive drive;
  private final Vision vision;
  private final Hopper hopper;
  private final Intake intake;
  private final Indexer indexer;
  private final PathGeneration pathGeneration;
  private final Outtake outtake;

  // -- Controllers --
  private final CommandJoystick driveJoystick =
      new CommandJoystick(ControllerConstants.JOYSTICK_LEFT_PORT);
  private final CommandJoystick steerJoystick =
      new CommandJoystick(ControllerConstants.JOYSTICK_MIDDLE_PORT);
  private final CommandJoystick operatorJoystick =
      new CommandJoystick(ControllerConstants.JOYSTICK_RIGHT_PORT);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RuntimeConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision = new Vision(new VisionIOLimelight());
        drive =
            new Drive(
                vision,
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        hopper = new Hopper(new HopperIOSparkMax());
        indexer = new Indexer(new IndexerIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        outtake = new Outtake();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision = new Vision(new VisionIO() {});
        drive =
            new Drive(
                vision,
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        hopper = new Hopper(new HopperIOSim());
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        outtake = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = new Vision(new VisionIO() {});
        drive =
            new Drive(
                vision,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIO() {});
        outtake = null;
    }

    pathGeneration = new PathGeneration();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamicDrive(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Turn SysId (Quasistatic Forward)",
        drive.sysIdQuasistaticRotate(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Turn SysId (Quasistatic Reverse)",
        drive.sysIdQuasistaticRotate(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Turn SysId (Dynamic Forward)", drive.sysIdDynamicRotate(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Turn SysId (Dynamic Reverse)", drive.sysIdDynamicRotate(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureJoystickBindings();

    PathfindingCommand.warmupCommand().schedule();
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
    // hopper.setDefaultCommand(
    //   Commands.run(() -> {
    //     if (driveJoystick.button(1).getAsBoolean()) {
    //       HopperIntakeCommands.startingExtension(hopper, intake);
    //     }else if(driveJoystick.button(2).getAsBoolean()){
    //       HopperIntakeCommands.hopperRetract(hopper, intake);
    //     }else if(driveJoystick.button(3).getAsBoolean()){
    //       HopperIntakeCommands.hopperExtend(hopper, intake);
    //     }
    //   }, hopper)
    // );
      // hopper.setDefaultCommand(HopperIntakeCommands.startingExtension(hopper, intake));
      driveJoystick.button(ControllerConstants.MAINHAND_BOTTOM_LEFT).onTrue(HopperIntakeCommands.startingExtension(hopper, intake));
      driveJoystick.button(ControllerConstants.MAINHAND_BOTTOM_MIDDLE).onTrue(HopperIntakeCommands.hopperRetract(hopper, intake));
      driveJoystick.button(ControllerConstants.MAINHAND_BOTTOM_RIGHT).onTrue(HopperIntakeCommands.hopperExtend(hopper, intake));
      driveJoystick.button(ControllerConstants.MAINHAND_TOP_LEFT).onTrue(HopperIntakeCommands.intakeWiggle(hopper, intake));

  

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

    System.out.println("Bindings configured");
    operatorJoystick.button(ControllerConstants.TRIGGER).onTrue(indexer.toggle());

    /**
    operatorJoystick.button(ControllerConstants.THUMB_BUTTON_RIGHT).onTrue(
      intake.runIntake()
    );
    */
    
    // Reset gyro to 0° when the drive joystick's trigger is pressed
    driveJoystick.button(ControllerConstants.TRIGGER).onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // // Should return the bot to its initial position
    // driveJoystick
    //     .button(ControllerConstants.TRIGGER)
    //     .whileTrue(
    //         Commands.runOnce(() -> System.out.println("Running path gen"))
    //             .andThen(pathGeneration.pathfindTo(Location.TEST_POSE)));

    // Might work better
    driveJoystick
        .button(ControllerConstants.THUMB_BUTTON_RIGHT)
        .onTrue(pathGeneration.pathfindToSimple(drive::getPose, Location.TEST_POSE, 0.0));

   // operatorJoystick.button(ControllerConstants.THUMB_BUTTON_RIGHT).whileTrue(
            //intake.runIntake().ignoringDisable(true));

    operatorJoystick.button(ControllerConstants.MAINHAND_BOTTOM_LEFT).whileTrue(outtake.sendBallsToShooter());

    operatorJoystick.button(ControllerConstants.THUMB_BUTTON_LEFT).whileTrue(outtake.aimWithJoystick(() -> operatorJoystick.getY()));

    operatorJoystick.button(ControllerConstants.THUMB_BUTTON_RIGHT).whileTrue(outtake.startFlywheel());
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
