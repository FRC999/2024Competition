// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveChassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTrajectoryNPoses extends SequentialCommandGroup {
  /** Creates a new AutonomousTrajectoryNPoses. */

  public AutonomousTrajectoryNPoses(Pose2d... poses) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //this(startPose, midPose, endPose, SwerveChassis.MAX_VELOCITY, SwerveChassis.MAX_ACCELERATION);
    //System.out.println("******* Start N Pose Trajectory ");
    List<PathPoint> plist = new ArrayList<>();

    for (Pose2d pose : poses) {
      plist.add(new PathPoint(pose.getTranslation(), new Rotation2d(0), pose.getRotation()));
    }
    addCommands(
      new RunTrajectorySequenceRobotAtStartPoint(
        PathPlanner.generatePath(
          new PathConstraints(SwerveChassis.MAX_VELOCITY, SwerveChassis.MAX_ACCELERATION),
          plist
        )
      )
    );
    
  }

  public AutonomousTrajectoryNPoses() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }
}
