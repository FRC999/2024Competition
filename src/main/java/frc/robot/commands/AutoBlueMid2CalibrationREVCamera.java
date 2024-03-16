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
import frc.robot.Constants.AutoConstants.autoPoses;
import frc.robot.lib.TrajectoryHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueMid2CalibrationREVCamera extends SequentialCommandGroup {
  /** Creates a new AutoBlueBottom3Notes. */
  public AutoBlueMid2CalibrationREVCamera() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new PrintCommand("Auto START"),
        // set yaw
        new InstantCommand(() -> RobotContainer.imuSubsystem.setYaw(
            autoPoses.BLUE_SPEAKER_MID.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose

        // ============================= SHOOTING PRELOADED NOTE ================================================================

        new ShootingGPM0Sequence(0) // shoot
     
            .andThen(new IntakeStop()), // stop intake

       
        // ============================= SPEAKER TO START POINT MID NOTE WHILE LOWERING THE INTAKE TO CAMERA ANGLE ================================================
       
        // drive to 2nd note pickup point
        (new AutonomousTrajectory2Poses( // drive to 1st note pickup
            autoPoses.BLUE_SPEAKER_MID.getPose(),
            autoPoses.BLUE_MID_RING_TAKE_START.getPose()))
        .alongWith(new ArmDownToNoteVision())
        ,

        // ============================= LOOK FOR THE NOTE< LOWER INTAKE ALL THE WAY AND DRIVE TO CORRECTED POSE FROM CAMERA TRAJECTORY =============================

        // Check if the note is visible, save the corrected pose endpoint
        new InstantCommand(() -> TrajectoryHelpers.setCorrectedPose(
            autoPoses.BLUE_MID_RING_TAKE_START.getPose(),
            autoPoses.BLUE_MID_RING_TAKE_END.getPose(),
            RobotContainer.photonVisionNoteHuntingSubsystem.xAngleToNote()))
        ,

        // pickup note, correct for the camera if possible
        (new DeferredCommand(
            () -> new AutonomousTrajectory2Poses( //DRIVE TO INTAKE MID NOTE
                autoPoses.BLUE_MID_RING_TAKE_START.getPose(), //START POSE
                TrajectoryHelpers.getCorrectedPose()) //CORRECTED END POSE

            , Set.of()).deadlineWith(
                new IntakeGrabNote()))
        .alongWith(
              new ArmDownToIntake())

        ,
        // stop intake
        new IntakeStop(), // in case we did not grab the note
        new ControllerRumbleStop(),

        // =============================  COME BACK 2 SPEAKER AND SHOOT ================================================================

        // go back to speaker
        new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce
                                        // shooting cycle
            autoPoses.BLUE_MID_RING_TAKE_END.getPose(),
            autoPoses.BLUE_SPEAKER_MID.getPose())
            .deadlineWith(
                new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0))),

        // shoot
        new ShootingGPM0Sequence(0),

        // cleanup
        new ShooterStop(), // stop shooter at the end
        new IntakeStop(), // stop intake
        new ControllerRumbleStop(),

        // drive out - do not really need to do that as already clossed the line
        // new AutonomousTrajectory2Poses( // drive to 2nd pickup point
        // autoPoses.BLUE_SPEAKER_MID.getPose(),
        // autoPoses.BLUE_MID_POS_OUT.getPose()),

        new PrintCommand("Auto DONE"));
  }
}