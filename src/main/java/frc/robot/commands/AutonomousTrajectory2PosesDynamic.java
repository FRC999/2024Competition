// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
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
