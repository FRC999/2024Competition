// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.autoPoses;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueBottomFarNote extends SequentialCommandGroup {
  /** Creates a new AutoBlueBottomFarNote. */
  public AutoBlueBottomFarNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
    //shoot preloaded note
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
        autoPoses.BLUE_SPEAKER_LOWER.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose
      new ShootingGPM0Sequence(0)   // shoot
        .andThen(new ShooterStop()) // stop shooter
        .andThen(new IntakeStop()), // stop intake

      new AutonomousTrajectory2Poses( // drive to W1
        autoPoses.BLUE_SPEAKER_LOWER.getPose(),
        autoPoses.BLUE_FAR_DRIVE_W1.getPose()),

      new AutonomousTrajectory2Poses( // drive to take start
        autoPoses.BLUE_FAR_DRIVE_W1.getPose(),
        autoPoses.BLUE_FAR_LOWER_TAKE_START.getPose()),

      new AutonomousTrajectory2Poses( // drive and run intake to pickup far note
        autoPoses.BLUE_FAR_LOWER_TAKE_START.getPose(),
        autoPoses.BLUE_FAR_LOWER_TAKE_END.getPose())
          .deadlineWith(new IntakeGrabNote()),
        
      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),

     
      (new AutonomousTrajectory2Poses( // drive to W1 and turn arm to angle preemptively to reduce shooting cycle
            autoPoses.BLUE_FAR_LOWER_TAKE_END.getPose(),
            autoPoses.BLUE_FAR_DRIVE_W1.getPose())
            .deadlineWith(
              new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0))
      )),


      new AutonomousTrajectory2Poses( // drive to start and shoot
        autoPoses.BLUE_FAR_DRIVE_W1.getPose(),
        autoPoses.BLUE_SPEAKER_LOWER.getPose())
          .deadlineWith(new ShootingGPM0Sequence(0))
              .andThen(new ShooterStop()) // stop shooter
              .andThen(new IntakeStop()) // stop intake
      
    );
  }
}
