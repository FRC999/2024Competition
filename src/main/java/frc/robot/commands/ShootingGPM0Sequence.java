// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingGPM0Sequence extends SequentialCommandGroup {
  /** Creates a new ShootingClose0. */
  public ShootingGPM0Sequence(double distance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Spin the shooter first
      new WaitCommand(0.5)
        .raceWith(
      new ShooterToSpeed(RobotContainer.gpmHelpers.getGPM0ShooterPower(distance))
        ),
      // Arm to angle, minimum wait 1.5s to the next step
      new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getGPM0Angle(distance)).
          alongWith(new WaitCommand(1.5)),
      // push note to shooter
      new IntakeRun(RobotContainer.gpmHelpers.getGPM0ShooterPower(
          RobotContainer.gpmHelpers.getGPM0IntakePower(distance))),
      // wait until the shooting is done
      new WaitCommand(1.5)
    );
  }
}