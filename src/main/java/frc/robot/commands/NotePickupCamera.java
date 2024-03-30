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
import frc.robot.Constants.AutoConstants.autoPoses;
import frc.robot.lib.TrajectoryHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NotePickupCamera extends SequentialCommandGroup {
  /** Creates a new NotePickupCamera. */
  public NotePickupCamera() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmDownToNoteVision2(),
      new InstantCommand(() ->
        RobotContainer.photonVisionNoteHuntingSubsystem.xAngleToNoteSaved()),

      new PrintCommand("NP-A:"+RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved()),

      new ConditionalCommand( // only shoot if picked up the note
        (new DeferredCommand(
            () -> new AutonomousTrajectory3Poses( // drive and run intake to pickup 1st note
                autoPoses.TARGET_NOTE_START.getPose(),
                TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation (
                  autoPoses.TARGET_NOTE_START.getPose(),
                  autoPoses.TARGET_NOTE_TAKE_START.getPose(),
                  RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved()
                ),
                TrajectoryHelpers.correctEndingPoseBasedOnNoteLocation (
                  autoPoses.TARGET_NOTE_TAKE_START.getPose(),
                  autoPoses.TARGET_NOTE_TAKE_END.getPose(),
                  RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved()
                )
              )

            , Set.of()
        ).alongWith(new ArmDownToIntake())
        ).raceWith(
            new IntakeGrabNote()),

        new PrintCommand("No Note Visible"),
        () -> RobotContainer.photonVisionNoteHuntingSubsystem.getxAngleToNoteSaved() != Double.NaN),

    // cleanup
    new ShooterStop(), // stop shooter
    new IntakeStop(), // stop intake
    new ControllerRumbleStop(),
    
    new PrintCommand("Note Pickup Command Done")

    );
  }
}
