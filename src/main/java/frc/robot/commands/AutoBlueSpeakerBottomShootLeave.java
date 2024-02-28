// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.BlueSpeakerBottomSideConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueSpeakerBottomShootLeave extends SequentialCommandGroup {
  /** Creates a new AutoBlueSpeakerBottomShootLeave. */

  private Pose2d startPose = new Pose2d(0.914, 4.315, new Rotation2d().fromDegrees(-60));
  private Pose2d endPose = new Pose2d(3.25, 1.312, new Rotation2d().fromDegrees(-60));

  public AutoBlueSpeakerBottomShootLeave() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(BlueSpeakerBottomSideConstants.yaw)), //steps to shoot preloaded note into speaker
      new ArmToAngleCommand(Constants.AutoConstants.BlueSpeakerBottomSideConstants.angle),
      new WaitCommand(5.0).deadlineWith(new ShooterToSpeed(Constants.AutoConstants.BlueSpeakerBottomSideConstants.power)),
      new AutonomousTrajectory2Poses(startPose, endPose)  //this will be our trajectory where we go from (0.46, 4.722478) to (3.25, 0.8) to leave community
      
    );

  }
}
