// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PosePrinter extends Command {
  /** Creates a new PosePrinter. */

  Supplier<Pose2d> poseSupplier;

  public PosePrinter(Supplier<Pose2d> p) {
    // Use addRequirements() here to declare subsystem dependencies.
    poseSupplier = p;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(poseSupplier.get());
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
    return true;
  }
}
