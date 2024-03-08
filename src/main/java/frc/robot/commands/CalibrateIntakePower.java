// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;

/**
 * Calibrate Intake Speed using Joystick's Z-slider
 */
public class CalibrateIntakePower extends Command {
  /** Creates a new CalibrateIntakeSpeed. */
  public CalibrateIntakePower() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("**** Calibrating Intake ...");
    double intakePower = RobotContainer.driveStick2.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.intakeSubsystem.runIntake(intakePower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakePower = RobotContainer.driveStick2.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.intakeSubsystem.runIntake(intakePower);
    SmartDashboard.putNumber("Calibration - Intake power", intakePower);
    SmartDashboard.putBoolean("Calibration - Intake sensor", RobotContainer.intakeSubsystem.isNoteInIntake());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
