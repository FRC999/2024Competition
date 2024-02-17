// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbUp extends InstantCommand {
  double power;

   public ClimbUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    this(Constants.ClimberConstants.CLIMBER_UP_DEFAULT_POWER);
  }

  
  public ClimbUp(double p) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climberSubsystem);

    this.power = p;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climberSubsystem.climbUP(power);
  }
}
