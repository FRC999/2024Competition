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
public class AutoCRNCRed2CenterFromBottom extends SequentialCommandGroup {
  /** Creates a new AutoREDCalibrationLowCenterFromBottom. */
  public AutoCRNCRed2CenterFromBottom() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
        autoPoses.RED_SPEAKER_LOWER.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose

    // ======================= SHOOT PRELOADED NOTE ===================================
      new ShootingGPM0Sequence(0)   // shoot
        .andThen(new IntakeStop()), // stop intake

    // ======================== DRIVE TO START TAKING NOTE, INTAKE TO NOTE SEEKING POSITION ==============================
      new AutonomousTrajectory3Poses( // drive to 1st note pickup
        autoPoses.RED_SPEAKER_LOWER.getPose(),
        autoPoses.RED_FAR_DRIVE_W1.getPose(),
        autoPoses.RED_FAR_LOWER_TAKE_START.getPose()
        )
        .alongWith(new ArmDownToNoteVision())
        ,
      // Just in case if the amr position is wrong - should finish pretty much instantaneously
        new ArmDownToNoteVision(),

        // Check if the note is visible, save the corrected pose endpoint
        new InstantCommand(() -> TrajectoryHelpers.setCorrectedPose(
            autoPoses.RED_FAR_LOWER_TAKE_START.getPose(),
            autoPoses.RED_FAR_LOWER_TAKE_END.getPose(),
            RobotContainer.photonVisionNoteHuntingSubsystem.xAngleToNote())),
        new DeferredCommand(
            () -> new PrintCommand("NoteAngle:" + RobotContainer.photonVisionNoteHuntingSubsystem.xAngleToNote()),
            Set.of()),
        new DeferredCommand(
            () -> new PrintCommand("CameraCorrectedPose:" + TrajectoryHelpers.getCorrectedPose()),
            Set.of()),

        // ========================== INTAKE LOW NOTE =============================================
        
        new ArmDownToIntake(), // now drop the intake all the way down

        // pickup note, correct for the camera if possible
        new DeferredCommand(
            () -> new AutonomousTrajectory2Poses( // drive and run intake to pickup 1st note
                autoPoses.RED_FAR_LOWER_TAKE_START.getPose(),
                //autoPoses.RED_FAR_LOWER_TAKE_END.getPose()
                TrajectoryHelpers.getCorrectedPose() // we corrected that via TrajectoryHelpers.setCorrectedPos
                )

            , Set.of()
        ).deadlineWith(
            new IntakeGrabNote())
        ,

        new IntakeStop(), // in case we did not grab the note
        new ControllerRumbleStop(),

    
      new ConditionalCommand( // only shoot if picked up the note

    // ====================== ON TRUE ================================================
        (new AutonomousTrajectory3Poses( // drive to original mid position and turn arm to angle preemptively to reduce shooting cycle
            autoPoses.RED_FAR_LOWER_TAKE_END.getPose(),
            autoPoses.RED_FAR_DRIVE_W1.getPose(),
            autoPoses.RED_SPEAKER_LOWER_2.getPose()
            )
            .deadlineWith(
              new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0))
            ))
              .andThen(new ShootingGPM0Sequence(0)) // shoot
              .andThen(new ShooterStop()) // stop shooter
              .andThen(new IntakeStop()) // stop intakE
              ,

      // ================== ON FALSE ====================================================
        new PrintCommand("Did not pickup MidField Note")
        .andThen(
          new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce shooting cycle
            autoPoses.RED_FAR_LOWER_TAKE_END.getPose(),
            autoPoses.RED_FAR_DRIVE_W1.getPose()
            )
        )

      // ================= CONDITION ====================================================================
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      )
    );
  }
}
