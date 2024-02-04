// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.LimeLightConstants;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.VisionHelpers;

public class LLDetectorSubsystem extends SubsystemBase implements VisionHelpers {
  /** Creates a new LLDetectorSubsystem. */
  public LLDetectorSubsystem() {}

  public boolean isNoteVisible() {
    return LimelightHelpers.getTV(LimeLightConstants.LLDetectorName);
  }

  public double getHorizontalAngleToNote() {
    if(isNoteVisible()) {
      return LimelightHelpers.getTX(LimeLightConstants.LLDetectorName);
    } else {
      return Double.NaN;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
