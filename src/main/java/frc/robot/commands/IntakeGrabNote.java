// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeGrabNote extends SequentialCommandGroup {
  /** Creates a new IntakeGrabNote. */
  public IntakeGrabNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("Grabbing Note..."),
      new IntakeRun(Intake.INTAKE_NOTE_GRAB_POWER).until(RobotContainer.intakeSubsystem::isNoteInIntake),
      new IntakeStop(),
      (new ControllerRumbleCommandDriver(0.25) // rumble driver controller if got the note in
        .onlyIf(RobotContainer.intakeSubsystem::isNoteInIntake))
          .alongWith(new LEDBlink().onlyIf(()->EnabledSubsystems.candle)) // Blink LEDs if installed
    );
  }
}
