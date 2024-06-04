// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TestStartTurningCommand extends Command {
  private double r = 0;
  private double previousVelocity = 0;
  private long previousTime;
  private double maxAcceleration;
  /** Creates a new TestStartTurningCommand. */
  public TestStartTurningCommand() {
    addRequirements(RobotContainer.driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousTime = System.currentTimeMillis();
    previousVelocity = RobotContainer.imuSubsystem.getTurnRate();
  }
  
  public double rotatePower() {
    return RobotContainer.driveStick.getRawAxis(3) * 3 + 3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    r = rotatePower();
    RobotContainer.driveSubsystem.drive(0, 0,
      r * Constants.SwerveChassis.MAX_ANGULAR_VELOCITY, true);
    long ct = System.currentTimeMillis();
    double cv =  RobotContainer.imuSubsystem .getTurnRate();
    SmartDashboard.putNumber("T-JP: " , r);
    SmartDashboard.putNumber("T-DPS: ", cv);
    double acc = (cv - previousVelocity) / (ct - previousTime);
    if (acc > maxAcceleration) {
      maxAcceleration = acc;
    }
    SmartDashboard.putNumber("T-A-DPSS: ", acc);
    SmartDashboard.putNumber("T-MA-DPSS: ", maxAcceleration);
    previousVelocity = cv;
    previousTime = ct;
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
