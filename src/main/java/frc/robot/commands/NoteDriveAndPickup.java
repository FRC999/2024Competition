// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.GPMConstants.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoteDriveAndPickup extends SequentialCommandGroup {
  /** Creates a new NoteDriveAndPickup. */
  public NoteDriveAndPickup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmDownToIntake()
        .alongWith(new IntakeRun(Intake.INTAKE_NOTE_GRAB_POWER))
        .alongWith(new NoteDriveCommand())
    );
  }
}
