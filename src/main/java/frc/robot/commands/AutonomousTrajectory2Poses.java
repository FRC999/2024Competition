// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveChassis;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTrajectory2Poses extends SequentialCommandGroup {

  Supplier<Pose2d> sp;
  Supplier<Pose2d> ep;

  /**
   * Run a trajectory between 2 poses with max velocity and acceleration
   * 
   * @param startPose
   * @param endPose
   */
  public AutonomousTrajectory2Poses(Supplier<Pose2d> startPose, Supplier<Pose2d> endPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this(startPose, endPose, SwerveChassis.MAX_VELOCITY, SwerveChassis.MAX_ACCELERATION);
  }

  /**
   * Run trajectory between two defined poses with specified max velocity and
   * acceleration
   * 
   * @param startPose
   * @param endPose
   * @param maxVelocity
   * @param maxAcceleration
   */
  public AutonomousTrajectory2Poses(Supplier<Pose2d> startPose, Supplier<Pose2d> endPose, double maxVelocity,
      double maxAcceleration) {

      sp = startPose;
      ep = endPose;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    System.out.println("*** Staring AutonomousTrajectory2Poses ***");

    if (sp == null || ep == null) { // do not have valid pose suppliers
      System.out.println("Pose supplier is NULL");
    } else {

        addCommands(
            new PrintCommand("=================== Driving from "),
            new PrintCommand(sp.get().toString()).onlyIf(()->sp.get()!=null),
            new PrintCommand("Driving to "),
            new PrintCommand(ep.get().toString()).onlyIf(()->ep.get()!=null),
            
            new ConditionalCommand (
              new RunTrajectorySequenceRobotAtStartPoint(
                PathPlanner.generatePath(
                  new PathConstraints(maxVelocity, maxAcceleration),
                  new PathPoint(sp.get().getTranslation(), new Rotation2d(0), sp.get().getRotation()), // position, heading
                  new PathPoint(ep.get().getTranslation(), new Rotation2d(0), ep.get().getRotation()) // position, heading
                )
              ),
              new PrintCommand("The pose is NULL. Not driving"),
              () -> { return sp != null && ep != null && sp.get()!=null && ep.get()!=null;}
            )
            
            //new PrintCommand("=================== Driving from "),
            //new PrintCommand(sp.get().toString()).onlyIf(()->sp.get()!=null),
            //new PrintCommand("Driving to "),
            //new PrintCommand(ep.get().toString()).onlyIf(()->ep.get()!=null)
          );

      }
    System.out.println("*** End AutonomousTrajectory2Poses ***");

  }
}
