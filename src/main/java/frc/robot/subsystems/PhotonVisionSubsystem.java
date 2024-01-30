// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.VisionHelpers;

public class PhotonVisionSubsystem extends SubsystemBase implements VisionHelpers {

  // private AprilTagFieldLayout aprilTagFieldLayout =
  // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // load 2024 field

  private String cameraName;
  private PhotonCamera camera;
  private PhotonPipelineResult aprilTagResult;
  private boolean aprilTagHasTargets;
  private List<PhotonTrackedTarget> aprilTagTargets;
  private PhotonTrackedTarget aprilTagBestTarget;
  private AprilTagFieldLayout aprilTagFieldLayout;
  // private PhotonPoseEstimator poseEstimator;
  private int fiducialID;
  private Transform3d robotToCam;
  private double aprilTagX, aprilTagY, aprilTagZAngle, aprilTagZ = -1;
  private Pose2d globalPoseEstimate = new Pose2d();
  private Transform3d fieldToCamera;
  // private Field2d apriltaField2d = new Field2d();

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem(String cameraName) {

    this.cameraName = cameraName;
    camera = new PhotonCamera(cameraName);
    aprilTagResult = new PhotonPipelineResult();
    aprilTagHasTargets = false;

    // aprilTagFieldLayout = new
    // AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
    // this.robotToCam = robotToCam;
    // poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

  }

  public Pose2d getRobotFieldPosePV() {

    aprilTagResult = camera.getLatestResult();
    aprilTagHasTargets = aprilTagResult.hasTargets();

    if (aprilTagHasTargets) {
      aprilTagTargets = aprilTagResult.getTargets();
      aprilTagBestTarget = aprilTagResult.getBestTarget();

      fiducialID = aprilTagBestTarget.getFiducialId();
      aprilTagX = aprilTagBestTarget.getBestCameraToTarget().getX();
      aprilTagY = aprilTagBestTarget.getBestCameraToTarget().getY();
      aprilTagZ = aprilTagBestTarget.getBestCameraToTarget().getZ();
      aprilTagZAngle = aprilTagBestTarget.getBestCameraToTarget().getRotation().getAngle();
      fieldToCamera = aprilTagResult.getMultiTagResult().estimatedPose.best;

      globalPoseEstimate = new Pose2d(fieldToCamera.getX(), fieldToCamera.getY(),
          new Rotation2d(fieldToCamera.getRotation().getX(), fieldToCamera.getRotation().getY()));
      // apriltaField2d.setRobotPose(globalPoseEstimate);

      return globalPoseEstimate;
    } else {
      return null;
    }

  }

  public boolean hasTargets() {
    return this.aprilTagHasTargets;
  }

  /**
   * Gets the Fiducial ID of the AprilTag.
   * 
   * @return The Fiducial ID.
   */
  public int getFiducialID() {
    return fiducialID;
  }

  /**
   * Gets the X coordinate of the AprilTag in meters.
   * 
   * @return The X coordinate.
   */
  public double getAprilTagX() {
    return aprilTagX;
  }

  /**
   * Gets the Y coordinate of the AprilTag in meters.
   * 
   * @return The Y coordinate.
   */
  public double getAprilTagY() {
    return aprilTagY;
  }

  /**
   * Gets the Z coordinate of the AprilTag in meters.
   * 
   * @return The Z coordinate.
   */
  public double getAprilTagZ() {
    return aprilTagZ;
  }

  /**
   * Gets the Z angle of the AprilTag in degrees.
   * 
   * @return The Z angle.
   */
  public double getAprilTagZAngle() {
    return aprilTagZAngle * (180 / Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
