// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.BlueSpeakerBottomSideConstants;
import frc.robot.Constants.AutoConstants.BlueSpeakerTopSideConstants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueSpeakerTopShootLeave extends SequentialCommandGroup {
  /** Creates a new AutoBlueSpeakerTopShootLeave. */
  private Pose2d startPose = new Pose2d(0.914, 6.764, new Rotation2d().fromDegrees(60));
  private Pose2d endPose = new Pose2d(3.25, 1.312, new Rotation2d().fromDegrees(60));

  public AutoBlueSpeakerTopShootLeave() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(BlueSpeakerTopSideConstants.yaw)), //steps to shoot preloaded note into speaker
      new ArmToAngleCommand(Constants.AutoConstants.BlueSpeakerTopSideConstants.angle),
      new ShooterToSpeed(Constants.AutoConstants.BlueSpeakerTopSideConstants.power),
      new RunTrajectorySequenceRobotAtStartPoint("BlueSpeakerBottomShootLeavePath") //this will be our trajectory where we go from (0.46, ) to (3.25, ) to leave community
    );
  }
}
