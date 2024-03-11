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
public class AutoBlueMid3CalLowREVNonStop extends SequentialCommandGroup {
  /** Creates a new AutoBlueBottom3Notes. */
  public AutoBlueMid3CalLowREVNonStop() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("Auto START"),

      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
        autoPoses.BLUE_SPEAKER_MID.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose

      new ShootingGPM0Sequence(0)   // shoot
        // .andThen(new ShooterStop()) // stop shooter
        .andThen(new IntakeStop()), // stop intake

      new ArmDownToIntake().alongWith( // intake down at the same time as driving
        new AutonomousTrajectory3Poses( // drive to 1st note pickup
            autoPoses.BLUE_SPEAKER_MID.getPose(),
            autoPoses.BLUE_MID_RING_TAKE_START.getPose(),
            autoPoses.BLUE_MID_RING_TAKE_END.getPose())
            .deadlineWith(
                new IntakeGrabNote())
      ),
      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),

      // Return back to MID and shoot
      new ConditionalCommand( // only shoot if picked up the note
        (new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce shooting cycle
            autoPoses.BLUE_MID_RING_TAKE_END.getPose(),
            autoPoses.BLUE_SPEAKER_MID.getPose())
            .deadlineWith(
              new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0))
            ))
              .andThen(new ShootingGPM0Sequence(0)) // shoot
              //.andThen(new ShooterStop()) // stop shooter
              .andThen(new IntakeStop()) // stop intake
              .andThen(
                new ArmDownToIntake().alongWith( // lower intake and also
                  new AutonomousTrajectory3Poses( // drive to 2nd pickup point, pickup 3rd note
                    autoPoses.BLUE_SPEAKER_MID.getPose(),
                      autoPoses.BLUE_LOWER_RING_TAKE_START.getPose(),
                      autoPoses.BLUE_LOWER_RING_TAKE_END.getPose())
                  .deadlineWith(
                    new IntakeGrabNote()
                  )
                )

              ),
        new PrintCommand("Did not pickup Mid Note")
          .andThen( // drive and pickup the 3rd note; intake should be down alreadu
            new ArmDownToIntake().alongWith( // lower intake and also
              new AutonomousTrajectory3Poses( // drive to 2nd pickup point, pickup 3rd note
                  autoPoses.BLUE_MID_RING_TAKE_END.getPose(),
                  autoPoses.BLUE_LOWER_RING_TAKE_START.getPose(),
                  autoPoses.BLUE_LOWER_RING_TAKE_END.getPose())
              .deadlineWith(
                new IntakeGrabNote()
                )
            )
          )
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ), // now we're at the 2nd pickup point


      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),
      new ConditionalCommand( // only shoot if picked up the note
        (new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce shooting cycle
            autoPoses.BLUE_LOWER_RING_TAKE_END.getPose(),
            autoPoses.BLUE_SPEAKER_MID.getPose())
            .deadlineWith(
              new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0))
            ))
              .andThen(new ShootingGPM0Sequence(0)) // shoot
              ,
        new PrintCommand("Did not pickup LOWER Note")
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ),

      // Cleanup
      new IntakeStop(),
      new ShooterStop(),
      new ArmDownToIntake(),
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