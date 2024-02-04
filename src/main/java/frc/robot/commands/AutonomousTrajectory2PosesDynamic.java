// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * This command will perform dynamic scheduling of the AutonomousTrajectory2Poses Sequence
 * This will allow us to run dynamic trajectories using RunTrajectorySequenceRobotAtStartPoint
 * and AutonomousTrajectoryRioCommand.
 * To do so, we pass the start/end poses as Pose Suppliers to this command, and do the pose determination
 * in "initialize()". We also create a new AutonomousTrajectory2Poses where we pass the poses.
 * That way we can drive trajectory between two poses we dynamically get in teleop, for instance,
 * from vision system.
 * Note that this class subclasses the InstantCommand, because we're only interested in doing initialize()
 */
public class AutonomousTrajectory2PosesDynamic extends InstantCommand {

  Supplier<Pose2d> startPose;
  Supplier<Pose2d> endPose;

  public AutonomousTrajectory2PosesDynamic(Supplier<Pose2d> sp, Supplier<Pose2d> ep) {
    // Use addRequirements() here to declare subsystem dependencies.
    startPose = sp;
    endPose = ep;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (startPose != null && endPose != null && startPose.get()!=null && endPose.get()!=null) {
      CommandScheduler.getInstance().schedule(
        //new PrintCommand(startPose.get().toString())
        //  .andThen(new PrintCommand(endPose.get().toString()))
        new AutonomousTrajectory2Poses(startPose.get(), endPose.get())
      );
    } else {
      CommandScheduler.getInstance().schedule(new PrintCommand("NULL pose, not driving"));
    }
  }
}
