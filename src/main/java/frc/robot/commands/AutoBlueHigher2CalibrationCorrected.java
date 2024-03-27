// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

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
public class AutoBlueHigher2CalibrationCorrected extends SequentialCommandGroup {
  /** Creates a new AutoBlueBottom3Notes. */
  public AutoBlueHigher2CalibrationCorrected() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        // set yaw
        new InstantCommand(() -> RobotContainer.imuSubsystem.setYaw(
            autoPoses.BLUE_SPEAKER_HIGHER.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose

        // ========================= SHOOT PRELOADED NOTE ===========================================
        new ShootingGPM0Sequence(0) // shoot
            .andThen(new IntakeStop()), // stop intake

        // ============================ DRIVE 2 HIGH NOTE =========================================
        new AutonomousTrajectory2Poses( // drive to 1st note pickup
            autoPoses.BLUE_SPEAKER_HIGHER.getPose(),
            autoPoses.BLUE_HIGHER_RING_TAKE_START_OPTIMIZED.getPose())
                .alongWith(new ArmDownToNoteVision())
            ,
      
        // Just in case if the arm position is wrong - should finish pretty much instantaneously
        new ArmDownToNoteVision(),

        // Check if the note is visible, save the corrected pose endpoint
        new InstantCommand(() -> TrajectoryHelpers.setCorrectedPose(
            autoPoses.BLUE_HIGHER_RING_TAKE_START_OPTIMIZED.getPose(),
            autoPoses.BLUE_HIGHER_RING_TAKE_END_OPTIMIZED.getPose(),
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
                autoPoses.BLUE_HIGHER_RING_TAKE_START_OPTIMIZED.getPose(),
                //autoPoses.BLUE_HIGHER_RING_TAKE_END.getPose()
                TrajectoryHelpers.getCorrectedPose() // we corrected that via TrajectoryHelpers.setCorrectedPos
                )

            , Set.of()
        ).deadlineWith(
            new IntakeGrabNote())
        ,
        // stop intake

        new IntakeStop(), // in case we did not grab the note
        new ControllerRumbleStop(),

        // ====================== GO BACK 2 SPEAKER =====================================

        // go back to speaker fror a corrected pose
        //new DeferredCommand(
        //    () -> new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce
        //                                // shooting cycle
        //    TrajectoryHelpers.getCorrectedPose(),
        //    autoPoses.BLUE_SPEAKER_HIGHER.getPose())
        //    , Set.of()
        //    )
        //    .deadlineWith(
        //        new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0))),

        // ================= SHOOT HIGH NOTE ====================================================
        //new ShootingGPM0Sequence(0),
        
        new ShootUsingLL(),

        // ================================= CLEAN UP =======================================
        new ShooterStop(), // stop shooter
        new IntakeStop(), // stop intake
        new ControllerRumbleStop(),

        // ========================= DRIVE OUT - NO NEED =============================================
        /*
        new AutonomousTrajectory2Poses( // drive to 2nd pickup point
            autoPoses.BLUE_SPEAKER_HIGHER.getPose(),
            autoPoses.BLUE_HIGHER_POS_OUT.getPose()),
        */
        new PrintCommand("Auto DONE"));
  }
}
