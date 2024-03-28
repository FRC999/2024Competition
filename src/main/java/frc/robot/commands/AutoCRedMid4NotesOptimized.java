// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants.autoPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCRedMid4NotesOptimized extends SequentialCommandGroup {
  /** Creates a new AutoREDBottom3Notes. */
  public AutoCRedMid4NotesOptimized() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
        autoPoses.RED_SPEAKER_MID.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose


      new ShootingGPM0Sequence(0)   // shoot
        .andThen(new ShooterStop()) // stop shooter
        .andThen(new IntakeStop()), // stop intake
      

      // ############ DRIVE TO MID PICKUP POINT ####################

      new AutonomousTrajectory2Poses( // drive to 1st note pickup
        autoPoses.RED_SPEAKER_MID.getPose(),
        autoPoses.RED_MID_RING_TAKE_START.getPose())
          .alongWith(new ArmDownToIntake())
        ,

      // ############ PICKUP MID NOTE ####################

      new AutonomousTrajectory2Poses( // drive and run intake to pickup 1st note
        autoPoses.RED_MID_RING_TAKE_START.getPose(),
        autoPoses.RED_MID_RING_TAKE_END.getPose())
          .deadlineWith(
              new IntakeGrabNote()
            ),
      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),

      // ############ SHOOT MID NOTE ####################

      new ConditionalCommand( // only shoot if picked up the note
        (new ShootingGPM0Sequence( // shoot at distance
            autoPoses.RED_SPEAKER_TAG.getPose().getTranslation().getDistance(
                 autoPoses.RED_MID_RING_TAKE_END.getPose().getTranslation())
            )
        ) 
            .andThen(new IntakeStop()) // stop intake
        ,
        new PrintCommand("Did not pickup Mid Note")
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ),

      // ############ DRIVE TO HIGH PICKUP POINT ################

      new AutonomousTrajectory2Poses(
          autoPoses.RED_MID_RING_TAKE_END.getPose(),
          autoPoses.RED_HIGHER_RING_TAKE_START_OPTIMIZED.getPose())
        .alongWith(new ArmDownToIntake())
      ,
      
      // ############ PICKUP HIGH NOTE ####################

      new AutonomousTrajectory2Poses( // drive and run intake to pickup 2nd note
        autoPoses.RED_HIGHER_RING_TAKE_START_OPTIMIZED.getPose(),
        autoPoses.RED_HIGHER_RING_TAKE_END_OPTIMIZED.getPose())
          .deadlineWith(
              new IntakeGrabNote()
            ),
      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),

      // ############ SHOOT HIGH NOTE ####################

      new ConditionalCommand( // only shoot if picked up the note
        (new ShootingGPM0Sequence( // shoot at distance
            autoPoses.RED_SPEAKER_TAG.getPose().getTranslation().getDistance(
                 autoPoses.RED_HIGHER_RING_TAKE_END_OPTIMIZED.getPose().getTranslation())
            )
        ) 
            .andThen(new IntakeStop()) // stop intake
        ,
        new PrintCommand("Did not pickup Higher Note")
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ),

      // ############ DRIVE TO LOW PICKUP POINT ################

      new AutonomousTrajectory2Poses(
          autoPoses.RED_HIGHER_RING_TAKE_END_OPTIMIZED.getPose(),
          autoPoses.RED_LOWER_RING_TAKE_START_OPTIMIZED.getPose())
        .alongWith(new ArmDownToIntake())
      ,

      // ############ PICKUP LOW NOTE ####################

      new AutonomousTrajectory2Poses( // drive and run intake to pickup 3rd note
        autoPoses.RED_LOWER_RING_TAKE_START_OPTIMIZED.getPose(),
        autoPoses.RED_LOWER_RING_TAKE_END_OPTIMIZED.getPose())
          .deadlineWith(
            new IntakeGrabNote()
          ),
      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),

      // ############ SHOOT LOW NOTE ####################
      
      new ConditionalCommand( // only shoot if picked up the note
        (new ShootingGPM0Sequence( // shoot at distance
            autoPoses.RED_SPEAKER_TAG.getPose().getTranslation().getDistance(
                 autoPoses.RED_LOWER_RING_TAKE_END_OPTIMIZED.getPose().getTranslation())
            )
        ) 
            .andThen(new IntakeStop()) // stop intake
        ,
        new PrintCommand("Did not pickup Lower Note")
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ),

      // ========================================== CLEAN UP ========================================================
      new ShooterStop(), // stop shooter
      new IntakeStop(), // stop intake
      new ControllerRumbleStop(),
      new PrintCommand("Auto DONE")

    );
  }
}
