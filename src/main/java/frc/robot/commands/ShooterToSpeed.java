// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.Shooter;

public class ShooterToSpeed extends Command {

  double speed;
  /** Creates a new ShooterToSpeed. */
  public ShooterToSpeed(double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    speed = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooterSubsystem.runShooterWithPower(
      RobotContainer.shooterSubsystem.convertShooterSpeedIntoShooterPower(speed));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Shooter is up to speed: " + speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.shooterSubsystem.getRightShooterMotorVelocity() - speed)
       <= Shooter.speedTolerance;
  }
}
