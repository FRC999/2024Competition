// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAllMotorsCommand extends InstantCommand {
  // Emergency stop of all motors


  public StopAllMotorsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem,
      RobotContainer.climberSubsystem,
      RobotContainer.intakeSubsystem,
      RobotContainer.shooterSubsystem,
      RobotContainer.driveSubsystem
      );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // Stop chassis
    for(int i = 0; i < 3; i++) {
      RobotContainer.driveSubsystem.stopAngleMotor(i);
      RobotContainer.driveSubsystem.stopDriveMotor(i);
    }

    //Stop intake motors
    RobotContainer.intakeSubsystem.stopIntake();

    //Stop shooter motors
    RobotContainer.shooterSubsystem.stopShooter();

    //Stop climber motor
    RobotContainer.climberSubsystem.stopClimbMotor();

    //Stop arm motor
    RobotContainer.armSubsystem.stopArmMotors();
  }

}
