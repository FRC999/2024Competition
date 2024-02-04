// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveChassis;

/**
 * This command drives a robot between two Poses. Note that poses are provided as
 * Pose2d objects and not as suppliers. Since the command sequence is formed in a constructor,
 * you cannot use this class directly to drive between two poses with unknown values.
 * However, this Command is scheduled by AutonomousTrajectory2PosesDynamic Command,
 * which works with dynamically determined poses.
 */
public class AutonomousTrajectory2Poses extends SequentialCommandGroup {
  /**
   * Run a trajectory between 2 poses with max velocity and acceleration
   * @param startPose
   * @param endPose
   */
  public AutonomousTrajectory2Poses( Pose2d startPose, Pose2d endPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this(startPose, endPose, SwerveChassis.MAX_VELOCITY, SwerveChassis.MAX_ACCELERATION);
  }

  /**
   * Run trajectory between two defined poses with specified max velocity and acceleration
   * @param startPose
   * @param endPose
   * @param maxVelocity
   * @param maxAcceleration
   */
  public AutonomousTrajectory2Poses( Pose2d startPose, Pose2d endPose, double maxVelocity, double maxAcceleration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    System.out.println("******* Start 2 Pose Trajectory ");
    System.out.println(startPose.toString());
    System.out.println(endPose.toString());

    addCommands(
      new RunTrajectorySequenceRobotAtStartPoint(
        PathPlanner.generatePath(
          new PathConstraints(maxVelocity, maxAcceleration),
          new PathPoint(startPose.getTranslation(), new Rotation2d(0), startPose.getRotation()), // position, heading
          new PathPoint(endPose.getTranslation(), new Rotation2d(0), endPose.getRotation()) // position, heading
        )
      )
    );
  }
}