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
public class AutoBlueMid3Notes2REVNonStop extends SequentialCommandGroup {
  /** Creates a new AutoBlueBottom3Notes. */
  public AutoBlueMid3Notes2REVNonStop() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.imuSubsystem.setYaw(
        autoPoses.BLUE_SPEAKER_MID.getPose().getRotation().getDegrees())), // set yaw to the one in the initial pose
// ========================= SHOOTING PRELOADED NOTE =========================/     
     
      new ShootingGPM0Sequence(0)   // shoot 
        .andThen(new IntakeStop()), // stop intake
      
// ========================= DRIVE TO INTAKE =========================/           
      new ArmDownToIntake(),
      (new AutonomousTrajectory3Poses( // drive to 1st note pickup
        autoPoses.BLUE_SPEAKER_MID.getPose(),
        autoPoses.BLUE_MID_RING_TAKE_START.getPose(),
        autoPoses.BLUE_MID_RING_TAKE_END.getPose()))
          .deadlineWith(new IntakeGrabNote()),
      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),

// ========================= CONDITIONS FOR SHOOTING =========================/       
      
      new ConditionalCommand( // only shoot if picked up the note
        
// ========================= IF NOTE PICKED UP =========================/           
        (new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce shooting cycle
            autoPoses.BLUE_MID_RING_TAKE_END.getPose(),
            autoPoses.BLUE_SPEAKER_MID.getPose())
            .deadlineWith(
              new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0))
            ))
              .andThen(new ShootingGPM0Sequence(0)) // shoot
              .andThen(new IntakeStop()) // stop intake
             // .andThen( new AutonomousTrajectory2Poses( // drive to 2nd pickup point
             //    autoPoses.BLUE_SPEAKER_MID.getPose(),
             //    autoPoses.BLUE_HIGHER_RING_TAKE_START.getPose())
              //)
              ,
// ========================= IF NOTE NOT PICKED UP : STOP =========================/           
        new PrintCommand("Did not pickup Mid Note")
         // .andThen(
         //   new AutonomousTrajectory2Poses( // drive to 2nd pickup point
         //        autoPoses.BLUE_MID_RING_TAKE_END.getPose(),
         //        autoPoses.BLUE_HIGHER_RING_TAKE_START.getPose())
         // )
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ), // now we're at the 2nd pickup point

// ========================= DRIVE TO 2ND NOTE PICK UP =========================/     
     
      new ArmDownToIntake(),
      new AutonomousTrajectory3Poses( // drive and run intake to pickup 2nd note
        autoPoses.BLUE_SPEAKER_MID.getPose(),
        autoPoses.BLUE_HIGHER_RING_TAKE_START.getPose(),
        autoPoses.BLUE_HIGHER_RING_TAKE_END.getPose())
          .alongWith(new IntakeGrabNote()),
      new IntakeStop(), // in case we did not grab the note
      new ControllerRumbleStop(),
      new ConditionalCommand( // only shoot if picked up the note
        (new AutonomousTrajectory2Poses( // drive to original mid position and turn arm to angle preemptively to reduce shooting cycle
            autoPoses.BLUE_HIGHER_RING_TAKE_END.getPose(),
            autoPoses.BLUE_SPEAKER_MID.getPose())
            .deadlineWith(
              new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(0))
            ))
              .andThen(new ShootingGPM0Sequence(0)) // shoot
              .andThen(new IntakeStop()) // stop intake
             // .andThen( new AutonomousTrajectory2Poses( // drive to 3rd pickup point
             //    autoPoses.BLUE_SPEAKER_MID.getPose(),
             //    autoPoses.BLUE_LOWER_RING_TAKE_START.getPose())
            //)
              ,

// ========================= DID NOT PICKUP HIGHER NOTE =========================/    

        new PrintCommand("Did not pickup Higher Note")
          //.andThen(
          //  new AutonomousTrajectory2Poses( // drive to 3rd pickup point
          //       autoPoses.BLUE_HIGHER_RING_TAKE_END.getPose(),
          //       autoPoses.BLUE_LOWER_RING_TAKE_START.getPose())
          //)
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ),
      new ArmDownToIntake(),

// ========================= GO TO 3RD PICK NOTE =========================/     
      
      new AutonomousTrajectory3Poses( // drive and run intake to pickup 3rd note
        autoPoses.BLUE_SPEAKER_MID.getPose(),
        autoPoses.BLUE_LOWER_RING_TAKE_START.getPose(),
        autoPoses.BLUE_LOWER_RING_TAKE_END.getPose())
          .alongWith(new IntakeGrabNote()),
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
              .andThen(new IntakeStop()) // stop intake
              .andThen( new AutonomousTrajectory2Poses( // drive OUT
                 autoPoses.BLUE_SPEAKER_MID.getPose(),
                 autoPoses.BLUE_MID_POS_OUT.getPose())
              ),
// ========================= DID NOT PICKUP HIGHER NOTE =========================/     
        new PrintCommand("Did not pickup Higher Note")
         // .andThen(
         //   new AutonomousTrajectory2Poses( // drive to 3rd pickup point
         //        autoPoses.BLUE_LOWER_RING_TAKE_END.getPose(),
         //        autoPoses.BLUE_LOWER_POS_OUT.getPose())
         // )
        , 
        RobotContainer.intakeSubsystem::isNoteInIntake
      ),
      //Cleanup
      new ShooterStop()

    );
  }
}
