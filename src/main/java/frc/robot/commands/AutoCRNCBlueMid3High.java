// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelpers;
import frc.robot.Constants.AutoConstants.autoPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCRNCBlueMid3High extends SequentialCommandGroup {
  /** Creates a new AutoBlueBottom3Notes. */
  public AutoCRNCBlueMid3High() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
        autoPoses.BLUE_SPEAKER_MID.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose

      // ========================= SHOOT PRELOADED NOTE ===========================================

      new ShootingGPM0Sequence(0)   // shoot
        .andThen(new IntakeStop()), // stop intake

      // ============================ DRIVE 2 MID NOTE =========================================

      new AutonomousTrajectory2Poses( // drive to 1st note pickup
          autoPoses.BLUE_SPEAKER_MID.getPose(),
          autoPoses.BLUE_MID_RING_TAKE_START.getPose())
            .alongWith(new ArmDownToNoteVision()),

      // Just in case if the arm position is wrong - should finish pretty much instantaneously
      new ArmDownToNoteVision(),

      // Check if the note is visible, save the corrected pose endpoint
      new InstantCommand(() -> TrajectoryHelpers.setCorrectedPose(
          autoPoses.BLUE_MID_RING_TAKE_START.getPose(),
          autoPoses.BLUE_MID_RING_TAKE_END.getPose(),
          RobotContainer.photonVisionNoteHuntingSubsystem.xAngleToNote())),
      new DeferredCommand(
          () -> new PrintCommand("NoteAngle:" + RobotContainer.photonVisionNoteHuntingSubsystem.xAngleToNote()),
          Set.of()),
      new DeferredCommand(
          () -> new PrintCommand("CameraCorrectedPose:" + TrajectoryHelpers.getCorrectedPose()),
          Set.of()),

      new ArmDownToIntake(), // now drop the intake all the way down

      // =========================== PICK UP MID NOTE ========================================

      // pickup note, correct for the camera if possible
      new DeferredCommand(
          () -> new AutonomousTrajectory2Poses( // drive and run intake to pickup 1st note
              autoPoses.BLUE_MID_RING_TAKE_START.getPose(),
              //autoPoses.BLUE_MID_RING_TAKE_END.getPose()
              TrajectoryHelpers.getCorrectedPose() // we corrected that via TrajectoryHelpers.setCorrectedPos
              )
           , Set.of()
      ).deadlineWith(
          new IntakeGrabNote())
      ,
      // stop intake

      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),

      new ConditionalCommand( // only shoot if picked up the note

        // go back to speaker fror a corrected pose
        (new DeferredCommand(
            () -> new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce
                                        // shooting cycle
            TrajectoryHelpers.getCorrectedPose(),
            autoPoses.BLUE_SPEAKER_MID.getPose())
            , Set.of()
            )
            .deadlineWith(
                new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0)))
        )
          .andThen(new ShootingGPM0Sequence(0)) // shoot
          .andThen(new IntakeStop()) // stop intake
          .andThen( new AutonomousTrajectory2Poses( // drive to 2nd pickup point
                 autoPoses.BLUE_SPEAKER_MID.getPose(),
                 autoPoses.BLUE_HIGHER_RING_TAKE_START.getPose())
              ) .alongWith(new ArmDownToNoteVision())
              ,

        new PrintCommand("Did not pickup Mid Note")
          .andThen(
            new AutonomousTrajectory2Poses( // drive to 2nd pickup point
                 autoPoses.BLUE_MID_RING_TAKE_END.getPose(),
                 autoPoses.BLUE_HIGHER_RING_TAKE_START.getPose())
            ) .alongWith(new ArmDownToNoteVision())
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ), // now we're at the 2nd pickup point

      // Just in case if the arm position is wrong - should finish pretty much instantaneously
      new ArmDownToNoteVision(),

      // Check if the note is visible, save the corrected pose endpoint
      new InstantCommand(() -> TrajectoryHelpers.setCorrectedPose(
          autoPoses.BLUE_HIGHER_RING_TAKE_START.getPose(),
          autoPoses.BLUE_HIGHER_RING_TAKE_END.getPose(),
          RobotContainer.photonVisionNoteHuntingSubsystem.xAngleToNote())),
      new DeferredCommand(
          () -> new PrintCommand("NoteAngle:" + RobotContainer.photonVisionNoteHuntingSubsystem.xAngleToNote()),
          Set.of()),
      new DeferredCommand(
          () -> new PrintCommand("CameraCorrectedPose:" + TrajectoryHelpers.getCorrectedPose()),
          Set.of()),

      new ArmDownToIntake(), // now drop the intake all the way down

      // =========================== PICK UP HIGH NOTE ========================================

      // pickup note, correct for the camera if possible
      new DeferredCommand(
          () -> new AutonomousTrajectory2Poses( // drive and run intake to pickup 1st note
              autoPoses.BLUE_HIGHER_RING_TAKE_START.getPose(),
              //autoPoses.BLUE_HIGHER_RING_TAKE_END.getPose()
              TrajectoryHelpers.getCorrectedPose() // we corrected that via TrajectoryHelpers.setCorrectedPos
              )
          , Set.of()
      ).deadlineWith(
          new IntakeGrabNote())
      ,

      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),

      new ConditionalCommand( // only shoot if picked up the note
        (
          new DeferredCommand(
            () -> new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce shooting cycle
            TrajectoryHelpers.getCorrectedPose(),
            autoPoses.BLUE_SPEAKER_MID.getPose())
            , Set.of()
            )
            .deadlineWith(
                new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0)))
          ) .andThen(new ShootingGPM0Sequence(0)) // shoot
        ,
        new PrintCommand("Did not pickup Higher Note")
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ),
      // ================================= CLEAN UP =======================================
      new ShooterStop(), // stop shooter
      new IntakeStop(), // stop intake
      new ControllerRumbleStop(),

      new PrintCommand("Auto DONE")
    );
  }
}

// Algorithm - 
// Change IMU
// Shoot
// Drive to mid-pickup-position, and intake down
// Start intake
// Drive over note to pick it up
// if picked up, drive back and shoot
// Drive to the second pickup position (upper)
// Start intake
// Drive over note to pick it up
// if picked up, drive back and shoot
// Drive to the third pickup position (lower)
// Start intake
// Drive over note to pick it up
// if picked up, drive back and shoot
// If note was not picked up, just go to the next one