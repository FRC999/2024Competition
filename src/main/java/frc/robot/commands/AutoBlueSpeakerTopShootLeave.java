// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.autoPoses;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueSpeakerTopShootLeave extends SequentialCommandGroup {
  /** Creates a new AutoBlueSpeakerTopShootLeave. */
  public AutoBlueSpeakerTopShootLeave() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
          autoPoses.BLUE_SPEAKER_HIGHER.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose
      new WaitCommand(10).deadlineWith(
        (new ShootingGPM0Sequence(0))
          .andThen(new ShooterStop()) // stop shooter
          .andThen(new IntakeStop()) ), // stop intake,   // shoot
      new AutonomousTrajectory2Poses(
        autoPoses.BLUE_SPEAKER_HIGHER.getPose(),
        autoPoses.BLUE_LOWER_POS_OUT.getPose()
      )  //this will be our trajectory where we go from (0.46, 4.722478) to (3.25, 0.8) to leave community
    );
  }
}
