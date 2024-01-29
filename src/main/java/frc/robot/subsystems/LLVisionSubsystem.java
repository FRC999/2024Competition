// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.LimeLightConstants;
import frc.robot.lib.LimelightHelpers;

public class LLVisionSubsystem extends SubsystemBase {

  /** Creates a new LLVisionSubsystem. 
   * Vision Subsystem based on LimeLight.
   * The coordinate tracking is always done with 0,0 on the lower blue side of the field.
  */
  public LLVisionSubsystem() {}

  public Pose2d getRobotFieldPoseLL() {
    if (LimelightHelpers.getTV(LimeLightConstants.LLAprilTagName)) { // LL target visible - meaning - see an Apriltag
      return LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName); //TODO: Check if the coordinates need to be translated to 0,0 of the blue lower corner
      // return LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName).relativeTo(LimeLightConstants.centerFieldPose); // check if this returns the right pose from 0,0
    } else {
      return null;
    }
  }

  public double getDistanceToRedSpeaker() {
    Pose2d robotPose = getRobotFieldPoseLL();
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Double.NaN;
    }
    return robotPose.getTranslation().getDistance(VisionConstants.redSpeakerTranslation);
  }

  public double getDistanceToBlueSpeaker() {
    Pose2d robotPose = getRobotFieldPoseLL();
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Double.NaN;
    }
    return robotPose.getTranslation().getDistance(VisionConstants.blueSpeakerTranslation);
  }

  public double getDistanceToRedAmp() {
    Pose2d robotPose = getRobotFieldPoseLL();
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Double.NaN;
    }
    return robotPose.getTranslation().getDistance(VisionConstants.redAmpTranslation);
  }

  public double getDistanceToBlueAmp() {
    Pose2d robotPose = getRobotFieldPoseLL();
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return Double.NaN;
    }
    return robotPose.getTranslation().getDistance(VisionConstants.blueAmpTranslation);
  }

  public Rotation2d getAngleToRedSpeaker() {
    Pose2d robotPose = getRobotFieldPoseLL();
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return null;
    }
    return robotPose.relativeTo(VisionConstants.redSpeakerPose).getRotation();
  }

  public Rotation2d getAngleToBlueSpeaker() {
    Pose2d robotPose = getRobotFieldPoseLL();
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return null;
    }
    return robotPose.relativeTo(VisionConstants.blueSpeakerPose).getRotation();
  }

  public Rotation2d getAngleToRedAmp() {
    Pose2d robotPose = getRobotFieldPoseLL();
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return null;
    }
    return robotPose.relativeTo(VisionConstants.redAmpPose).getRotation();
  }

  public Rotation2d getAngleToBlueAmp() {
    Pose2d robotPose = getRobotFieldPoseLL();
    if (robotPose == null) {  // return NaN if the camera cannot determine the pose
      return null;
    }
    return robotPose.relativeTo(VisionConstants.blueAmpPose).getRotation();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
