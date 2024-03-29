// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ControllerRumbleCommandDriver extends SequentialCommandGroup {
  /** Creates a new ControllerRumbleCommand. */
  public ControllerRumbleCommandDriver(double seconds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.xboxDriveController.setRumble(RumbleType.kBothRumble, 1) ), // start rumble half-intensity
      new WaitCommand(seconds),
      new InstantCommand( () -> RobotContainer.xboxDriveController.setRumble(RumbleType.kBothRumble, 0) ) // stop rumble
    );
  }
}
