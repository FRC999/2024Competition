// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

public class CalibrateArmPowerFF extends Command {
  /** Creates a new CalibrateArmPower. */
  public CalibrateArmPowerFF() {
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double armPower = RobotContainer.driveStick.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.armSubsystem.runArmMotors(armPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armPower = RobotContainer.driveStick.getRawAxis(OIConstants.CALIBRATION_JOYSTICK_SLIDER_AXLE);
    RobotContainer.armSubsystem.runArmMotors(armPower);
    SmartDashboard.putNumber("Calibration - Arm power FF", armPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}