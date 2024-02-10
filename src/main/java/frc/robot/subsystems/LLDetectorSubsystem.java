// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.LimeLightConstants;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.VisionHelpers;

public class LLDetectorSubsystem extends SubsystemBase implements VisionHelpers {
  private final double[] doNotMoveRobot = {0,0};
  /** Creates a new LLDetectorSubsystem. */
  public LLDetectorSubsystem() {}

  public boolean isNoteVisible() {
    return LimelightHelpers.getTV(LimeLightConstants.LLDetectorName);
  }

  public double getHorizontalSide(){
    return LimelightHelpers.getThor(LimeLightConstants.LLDetectorName);
  }

  public double getHorizontalOffset(){
    return LimelightHelpers.getTX(LimeLightConstants.LLDetectorName);
  }

  public double getVerticalOffset(){
    return LimelightHelpers.getTY(LimeLightConstants.LLDetectorName);
  }

  public double getHorizontalAngleToNote() {
    if(isNoteVisible()) {
      return LimelightHelpers.getTX(LimeLightConstants.LLDetectorName);
    } else {
      return Double.NaN;
    }
  }

  public double[] driveToNotePowers() {
    double horizontalAngleToNote = getHorizontalAngleToNote();
    if(horizontalAngleToNote == Double.NaN) {
      return doNotMoveRobot;
    }
    double x = Math.sin(getHorizontalOffset())*Constants.VisionConstants.LimeLightConstants.VELOCITY_TO_AUTO_NOTE;
    double y = Math.cos(getHorizontalOffset())*Constants.VisionConstants.LimeLightConstants.VELOCITY_TO_AUTO_NOTE;
    double[] noteDrivingInstructions = {x,y};
    return noteDrivingInstructions ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
