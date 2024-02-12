// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.VisionConstants.LimeLightConstants;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.VisionHelpers;

public class LLVisionSubsystem extends SubsystemBase implements VisionHelpers {
  public static double joystickDirectionDegrees;
  private Pose2d nullPose = new Pose2d(0,0,new Rotation2d(0));
  // position of the LL relative to the center of the chassis; the X value may need to be remeasured if used to correct odometry
  private Transform2d cameraPoseInRobotSpace = new Transform2d(-SwerveChassis.CHASSIS_OUTER_DIMENSIONS_X/2.0,0,new Rotation2d(Math.PI));

  /** Creates a new LLVisionSubsystem. 
   * Vision Subsystem based on LimeLight.
   * The coordinate tracking is always done with 0,0 on the lower blue side of the field.
  */
  public LLVisionSubsystem() {}

  public Pose2d getRobotFieldPoseLL() {
    if (LimelightHelpers.getTV(LimeLightConstants.LLAprilTagName)) { // LL target visible - meaning - see an Apriltag


      //System.out.println("TESTPOSE:"+LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName).transformBy(cameraPoseInRobotSpace));
      // We need to transform the camera pose to the chassis pose
      return LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName).transformBy(cameraPoseInRobotSpace); //TODO: Check if the coordinates need to be translated to 0,0 of the blue lower corner
      // return LimelightHelpers.getBotPose2d_wpiBlue(LimeLightConstants.LLAprilTagName).relativeTo(LimeLightConstants.centerFieldPose); // check if this returns the right pose from 0,0
    } else {
      return null; //TODO: Consider changing this class to return Optional<Pose2d>
    }
  }

  public boolean isApriltagVisible() {
    return LimelightHelpers.getTV(LimeLightConstants.LLAprilTagName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
