// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * Drive to note if you see it.
 * We do not re-check if we still see it, as we drive
 */
public class NoteDriveCommand extends Command {
  private double[] noteDriveDirections;
  /** Creates a new RobotToTarget. */
  public NoteDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get X and Y of the drive to note related to the direction
    noteDriveDirections = RobotContainer.llDetectorSubsystem.driveToNotePowers();
    RobotContainer.driveSubsystem.drive(noteDriveDirections[0], noteDriveDirections[1], 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
