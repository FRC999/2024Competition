// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTablesSubsystem extends SubsystemBase {
  /** Creates a new NetworkTablesSystem. */
  private NetworkTableInstance ntInst;
  public NetworkTablesSubsystem() {
    ntInst = NetworkTableInstance.getDefault();
  }
  public Pose2d getLimelightLeftRobotPose() {

    double[] robotPoseArray = ntInst.getTable("limelight-back").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    return new Pose2d(robotPoseArray[0],robotPoseArray[1],new Rotation2d(Units.degreesToRadians(robotPoseArray[5])));
  }

  public Pose2d getLimelightRightRobotPose() {

    double[] robotPoseArray = ntInst.getTable("limelight-front").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    return new Pose2d(robotPoseArray[0],robotPoseArray[1],new Rotation2d(Units.degreesToRadians(robotPoseArray[5])));
  }

  public boolean isLeftTargetAcquired() {
    return ntInst.getTable("limelight-back").getEntry("tv").getDouble(0) == 1.0; // return true if the target visible
  }
  public boolean isRightTargetAcquired() {
    return ntInst.getTable("limelight-front").getEntry("tv").getDouble(0) == 1.0; // return true if the target visible
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
