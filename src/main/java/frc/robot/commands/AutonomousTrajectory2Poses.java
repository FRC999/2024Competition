// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveChassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTrajectory2Poses extends SequentialCommandGroup {
  /**
   * Run a trajectory between 2 poses with max velocity and acceleration
   * @param startPose
   * @param endPose
   */
  public AutonomousTrajectory2Poses( Supplier<Pose2d> startPose, Supplier<Pose2d> endPose) {
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
  public AutonomousTrajectory2Poses( Supplier<Pose2d> startPose, Supplier<Pose2d> endPose, double maxVelocity, double maxAcceleration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousTrajectoryRioCommand(
        PathPlanner.generatePath(
          new PathConstraints(maxVelocity, maxAcceleration),
          new PathPoint(startPose.get().getTranslation(), startPose.get().getRotation()), // position, heading
          new PathPoint(endPose.get().getTranslation(), endPose.get().getRotation()) // position, heading
        )
      )
    );
  }
}
