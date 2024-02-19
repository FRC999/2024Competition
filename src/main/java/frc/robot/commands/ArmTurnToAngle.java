// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.Arm.ArmPIDConstants;

public class ArmTurnToAngle extends Command {

  DoubleSupplier angleSupplier;
  double angle;
  /** Creates a new TurnArmToAngle. */
  public ArmTurnToAngle(DoubleSupplier as) {
    // Use addRequirements() here to declare subsystem dependencies.
    angleSupplier = as;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = angleSupplier.getAsDouble();
    RobotContainer.gpmSubsystem.setArmMotorAnglesSI(angle);
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
    return (Math.abs(angle-RobotContainer.gpmSubsystem.getArmAngleSI())<ArmPIDConstants.anglePIDTolerance);
  }
}
