// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTablesSubsystem extends SubsystemBase {
  /** Creates a new NetworkTablesSystem. */
  private NetworkTableInstance ntInst;
  private String botpose = getAllianceKey();
  
  public NetworkTablesSubsystem() {
    ntInst = NetworkTableInstance.getDefault();
  }

  public String getAllianceKey() {
     Optional<Alliance> alliance = DriverStation.getAlliance();
    
    /*
     * Get your current alliance and depending on what alliance you are you have to
     * set the
     * key for getEntry as botpose_wpi + alliance
     */

    if (alliance.isPresent()) {
     if(alliance.get() == DriverStation.Alliance.Blue){

  
      botpose = "botpose_wpiblue";
    } else {
      botpose = "botpose_wpired";
    }
  }
    System.out.println(botpose);
    return botpose;
  }

  

  public Pose2d getLimelightBackRobotPose() {
    double[] robotPoseArray = ntInst.getTable("limelight-back").getEntry(botpose).getDoubleArray(new double[6]);

    return new Pose2d(robotPoseArray[0], robotPoseArray[1], new Rotation2d(Units.degreesToRadians(robotPoseArray[5])));
  }

  public Pose2d getLimelightFrontRobotPose() {

    double[] robotPoseArray = ntInst.getTable("limelight-front").getEntry(botpose).getDoubleArray(new double[6]);

    return new Pose2d(robotPoseArray[0], robotPoseArray[1], new Rotation2d(Units.degreesToRadians(robotPoseArray[5])));
  }

  public boolean isBackTargetAcquired() {
    return ntInst.getTable("limelight-back").getEntry("tv").getDouble(0) == 1.0; // return true if the target visible
  }

  public boolean isFrontTargetAcquired() {
    return ntInst.getTable("limelight-front").getEntry("tv").getDouble(0) == 1.0; // return true if the target visible
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
