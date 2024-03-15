// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants.LimeLightConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveToClimbingPosition extends SequentialCommandGroup {
  /** Creates a new AutoDriveToClimbingPosition. */
  public AutoDriveToClimbingPosition() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        // ======== DRIVE FROM CURRENT POSE TO PRE_CLIMB POSE
        new DeferredCommand(
            () -> new AutonomousTrajectory2Poses( //DRIVE TO INTAKE MID NOTE
              RobotContainer.llVisionSubsystem.getRobotFieldPoseLLBeforeClimb(), //START POSE
              LimeLightConstants.robotPreClimbingPoses.get(RobotContainer.llVisionSubsystem.getFiducialId()) // END POSE
              )
            , Set.of()
          )
        .andThen( new ArmUpForClimb() ) // === ONCE IN PRE_CLIMB POSE, RAISE ARM

        // ====== DRIVE FROM PRE_CLIMB POSE TO CHAIN ======
        .andThen(
          new AutonomousTrajectory2Poses(
            LimeLightConstants.robotPreClimbingPoses.get(RobotContainer.llVisionSubsystem.getFiducialId()),
            LimeLightConstants.robotClimbingPoses.get(RobotContainer.llVisionSubsystem.getFiducialId()))
        )
        ,
        new PrintCommand("No Climb Apriltag visible"),
        RobotContainer.llVisionSubsystem::isApriltagClimbVisible
      )
    );
  }
}
