package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathGenerationConstants;
import frc.robot.Constants.PathGenerationConstants.Location;

import java.util.HashMap;
import java.util.Map;

public class PathGeneration {

  private final Map<String, Pose2d> customLocations = new HashMap<>();

  public PathGeneration() {}

  /**
   * Generates a command to pathfind to a specific known location.
   *
   * @param location The target location of interest.
   * @return The pathfinding command.
   */
  public Command pathfindTo(Location location) {
    return pathfindTo(location.getPose(), PathGenerationConstants.DEFAULT_CONSTRAINTS);
  }

  /**
   * Generates a command to pathfind to a specific known location with custom constraints.
   *
   * @param location The target location of interest.
   * @param constraints The path constraints to use.
   * @return The pathfinding command.
   */
  public Command pathfindTo(Location location, PathConstraints constraints) {
    return pathfindTo(location.getPose(), constraints);
  }

  /**
   * Generates a command to pathfind to an arbitrary pose.
   *
   * @param targetPose The target pose on the field.
   * @return The pathfinding command using default constraints.
   */
  public Command pathfindTo(Pose2d targetPose) {
    return pathfindTo(targetPose, PathGenerationConstants.DEFAULT_CONSTRAINTS);
  }

  /**
   * Generates a command to pathfind to an arbitrary pose with custom constraints.
   *
   * @param targetPose The target pose on the field.
   * @param constraints The path constraints to use.
   * @return The pathfinding command.
   */
  public Command pathfindTo(Pose2d targetPose, PathConstraints constraints) {
    // 0.0 is the goal end velocity (stop at the end)
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  /**
   * Generates a command to pathfind to a specific known location.
   *
   * @param location The target location of interest.
   * @return The pathfinding command.
   */
  public Command pathfindToSimple(Location location, double endVelocity_MPS) {
    return pathfindToSimple(location.getPose(), PathGenerationConstants.DEFAULT_CONSTRAINTS, endVelocity_MPS);
  }

  /**
   * Generates a command to pathfind to a specific known location with custom constraints.
   *
   * @param location The target location of interest.
   * @param constraints The path constraints to use.
   * @return The pathfinding command.
   */
  public Command pathfindToSimple(Location location, PathConstraints constraints, double endVelocity_MPS) {
    return pathfindToSimple(location.getPose(), constraints, endVelocity_MPS);
  }

  /**
   * Generates a command to pathfind to an arbitrary pose.
   *
   * @param targetPose The target pose on the field.
   * @return The pathfinding command using default constraints.
   */
  public Command pathfindToSimple(Pose2d targetPose) {
    return pathfindToSimple(targetPose, PathGenerationConstants.DEFAULT_CONSTRAINTS, 0);
  }

  /**
   * Generates a command to pathfind to an arbitrary pose with custom constraints.
   *
   * @param targetPose The target pose on the field.
   * @param constraints The path constraints to use.
   * @return The pathfinding command.
   */
  public Command pathfindToSimple(Pose2d targetPose, PathConstraints constraints, double endVelocity_MPS) {
    // 0.0 is the goal end velocity (stop at the end)
    return AutoBuilder.followPath(new PathPlannerPath(
      PathPlannerPath.waypointsFromPoses(
        new Pose2d(AutoBuilder.getCurrentPose().getTranslation(), targetPose.getTranslation().minus(AutoBuilder.getCurrentPose().getTranslation()).getAngle()),
        new Pose2d(targetPose.getTranslation(), targetPose.getTranslation().minus(AutoBuilder.getCurrentPose().getTranslation()).getAngle())
      ), constraints, null, new GoalEndState(endVelocity_MPS, targetPose.getRotation()))
    );
  }

  /**
   * Generates a command to pathfind to a location while running another command in parallel.
   * The parallel command will run until the pathfinding is complete.
   *
   * @param location The target location.
   * @param parallelCommand The command to run while moving.
   * @return A ParallelDeadlineGroup containing the pathfinding command and the parallel command.
   */
  public Command pathfindToWithHook(Location location, Command parallelCommand) {
    return pathfindToWithHook(location.getPose(), PathGenerationConstants.DEFAULT_CONSTRAINTS, parallelCommand);
  }

  /**
   * Generates a command to pathfind to a pose while running another command in parallel.
   * The parallel command will run until the pathfinding is complete.
   *
   * @param targetPose The target pose.
   * @param constraints Path constraints.
   * @param parallelCommand The command to run while moving.
   * @return A ParallelDeadlineGroup containing the pathfinding command and the parallel command.
   */
  public Command pathfindToWithHook(
      Pose2d targetPose, PathConstraints constraints, Command parallelCommand) {
    return Commands.deadline(pathfindTo(targetPose, constraints), parallelCommand);
  }

  /**
   * Adds a custom location of interest at runtime.
   *
   * @param name Name of the location.
   * @param pose Pose of the location.
   */
  public void addCustomLocation(String name, Pose2d pose) {
    customLocations.put(name, pose);
  }

  /**
   * Gets a custom location by name.
   *
   * @param name Name of the location.
   * @return The pose, or null if not found.
   */
  public Pose2d getCustomLocation(String name) {
    return customLocations.get(name);
  }
}
