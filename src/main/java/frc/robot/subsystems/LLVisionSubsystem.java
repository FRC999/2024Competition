// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants.autoPoses;
import frc.robot.Constants.VisionConstants.LimeLightConstants;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.VisionHelpers;

public class LLVisionSubsystem extends SubsystemBase implements VisionHelpers {
  public static double joystickDirectionDegrees;
  public static Pose2d robotCurrentPoseBeforeClimb;
  public static double distanceToShoot = -1;

  /** Creates a new LLVisionSubsystem. 
   * AprilTag Vision Subsystem based on LimeLight.
   * The coordinate tracking is always done with 0,0 on the lower blue side of the field.
  */
  public LLVisionSubsystem() {}

  public Pose2d getRobotFieldPoseLL() {
    if (LimelightHelpers.getTV(LimeLightConstants.LLAprilTagName)) { // LL target visible - meaning - see an Apriltag
      return LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName).transformBy(LimeLightConstants.cameraToRobotTransform); //TODO: Check if the coordinates need to be translated to 0,0 of the blue lower corner
      // return LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName).relativeTo(LimeLightConstants.centerFieldPose); // check if this returns the right pose from 0,0
    } else {
      return null; //TODO: Consider changing this class to return Optional<Pose2d>
    }
  }

  public boolean isApriltagVisible() {
    return LimelightHelpers.getTV(LimeLightConstants.LLAprilTagName);
  }

  public boolean isApriltagClimbVisible() {
    if (isApriltagVisible()) {
      // save the robot pose if we see LL
      robotCurrentPoseBeforeClimb = LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName);
      //.transformBy(LimeLightConstants.cameraToRobotTransform);
      double fid = getFiducialId();
      return fid>=11.0 && fid <=16.0;
    }
    return false; // if no apriltag visible
  }

  public double getShootingDistance(Pose2d pose) {
    if (RobotContainer.isAlianceRed) {

      // alex test
      //System.out.println("RP:"+pose);
      return autoPoses.RED_SPEAKER_TAG.getPose().getTranslation().getDistance(
         pose.getTranslation()
      );
    } else {
        return autoPoses.BLUE_SPEAKER_TAG.getPose().getTranslation().getDistance(
         pose.getTranslation()
      );
    }
  }

  public double getShootingDistance() {
    if (isApriltagVisible()) {
      distanceToShoot = Math.abs(getShootingDistance(getRobotFieldPoseLL())); // use absolute distance, so works on both RED and BLUE
    } else {
      distanceToShoot = -1;
    }

    // alex test
    //System.out.println("ATV0-D:"+distanceToShoot);
    return distanceToShoot;
  }

  public Pose2d getRobotFieldPoseLLBeforeClimb() {
    return robotCurrentPoseBeforeClimb;
  }

  public double getFiducialId() {
    return LimelightHelpers.getFiducialID(LimeLightConstants.LLAprilTagName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
