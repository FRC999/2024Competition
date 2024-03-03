// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants.autoPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRedSpeakerMidShootLeave extends SequentialCommandGroup {
  /** Creates a new AutoRedSpeakerMidShootLeave. */
  public AutoRedSpeakerMidShootLeave() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("Starting AutoRedSpeakerMidShoot"),
       new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
        autoPoses.RED_SPEAKER_MID.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose
      new PrintCommand("Auto yaw set"),
      new WaitCommand(20).raceWith(
        (new ShootingGPM0Sequence(0))
          .andThen(new ShooterStop()) // stop shooter
          .andThen(new IntakeStop())) // stop intake
        ,   // shoot
      new PrintCommand("Shooting done"),
      new WaitCommand(1.0), //wait 5 seconds to give time for other ; ADJUST BASED ON HOW LONG IT TAKES TO LEAVE COMMUNITY
      new PrintCommand("Wait done"),
      new AutonomousTrajectory2Poses(
        autoPoses.RED_SPEAKER_MID.getPose(),
        autoPoses.RED_MID_POS_OUT.getPose()
      ),
      new PrintCommand("Trajectory done")
        //this will be our trajectory where we go from (13.789, 5.514) to (11.453, 5.540) to leave community
    );
  }
}
