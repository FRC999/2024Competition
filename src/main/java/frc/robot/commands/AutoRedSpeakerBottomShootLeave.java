// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants.autoPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRedSpeakerBottomShootLeave extends SequentialCommandGroup {
  /** Creates a new AutoRedSpeakerBottomShootLeave. */
  public AutoRedSpeakerBottomShootLeave() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
          autoPoses.RED_SPEAKER_LOWER.getPose().getRotation().getDegrees())), // set yaw to the start pose yaw
      new WaitCommand(20).deadlineWith(
        new ShootingGPM0Sequence(0)),   // shoot from closest shooting distance
      new AutonomousTrajectory2Poses(
        autoPoses.RED_SPEAKER_LOWER.getPose(),
        autoPoses.RED_LOWER_POS_OUT.getPose()
      )  //this will be our trajectory where we go from (13.992, 4.667) to (11.453, 1.32) to leave community
    );
  }
}
