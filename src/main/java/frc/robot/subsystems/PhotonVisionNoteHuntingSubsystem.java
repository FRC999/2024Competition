// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnabledSubsystems;

public class PhotonVisionNoteHuntingSubsystem extends SubsystemBase {

  PhotonCamera camera;
  private boolean cameraConnected = true;
  private double xAngleToNoteSaved = 0;
  private double yAngleToNoteSaved = 0;

  private double heightOfCamera = 0.232;
  private double cameraPitchOffset = 2.519;
  private double centerOfRobotToCamera = 0.28;

  /** Creates a new PhotonVisionNoteHuntingSubsystem. */
  public PhotonVisionNoteHuntingSubsystem(String cameraName) {
      if (! EnabledSubsystems.noteHuntingCamera) { return; }

      try {
      camera = new PhotonCamera(cameraName); // TODO: check camera name
      cameraConnected = camera.isConnected();
      camera.setDriverMode(false);
      } catch(Exception e) {
        cameraConnected =  false;
      }

      System.out.println("Camera connected:"+camera.isConnected());
  }

  public boolean isNoteDetected() {

    //if (!cameraConnected) {
    //  return false;
    //}

    try {
      var result = camera.getLatestResult();
      return cameraConnected && result.hasTargets(); // if camera is not connected, return FALSE
    } catch (Exception e) {
      return false;
    }
  }

  public double xAngleToNote() {

    //if (!cameraConnected) {
    //  return 0;
    //}

    try {
      var result = camera.getLatestResult();

      if (!result.hasTargets()) { // I do not see notes return 0 angle
        return 0;
      }
      PhotonTrackedTarget target = result.getBestTarget();
      return -target.getYaw(); // getYaw here returns positive right
    } catch (Exception e) {
      return 0;
    }

  }

  public double yAngleToNote() {

    //if (!cameraConnected) {
    //  return 0;
    //}

    try {
      var result = camera.getLatestResult();

      if (!result.hasTargets()) { // I do not see notes return 0 angle
        return 0;
      }
      PhotonTrackedTarget target = result.getBestTarget();
      return target.getPitch(); 
    } catch (Exception e) {
      return 0;
    }

  }

  public void xAngleToNoteSaved() {

    //if (!cameraConnected) {
    //  return 0;
    //}

    try {
      var result = camera.getLatestResult();

      if (!result.hasTargets()) { // I do not see notes return 0 angle
        xAngleToNoteSaved = Double.NaN;
      }
      PhotonTrackedTarget target = result.getBestTarget();
      xAngleToNoteSaved = -target.getYaw(); // getYaw here returns positive right
    } catch (Exception e) {
        xAngleToNoteSaved = Double.NaN;
    }

  }

   public void xyAngleToNoteSaved() {

    //if (!cameraConnected) {
    //  return 0;
    //}

    try {
      var result = camera.getLatestResult();

      if (!result.hasTargets()) { // I do not see notes return 0 angle
        xAngleToNoteSaved = Double.NaN;
        yAngleToNoteSaved = Double.NaN;
      }
      PhotonTrackedTarget target = result.getBestTarget();
      xAngleToNoteSaved = -target.getYaw(); // getYaw here returns negative right
      yAngleToNoteSaved = target.getPitch();
    } catch (Exception e) {
        xAngleToNoteSaved = Double.NaN;
        yAngleToNoteSaved = Double.NaN;
    }

  }

  public double getxAngleToNoteSaved(){

    System.out.println("X : " + xAngleToNoteSaved);
    return xAngleToNoteSaved;
    
  }

  public double getyAngleToNoteSaved(){

    System.out.println("y : " + yAngleToNoteSaved);
    return yAngleToNoteSaved;
    
  }

  public void resetXAngleToNoteSaved() {
    xAngleToNoteSaved = 0;
  }

  public void resetYAngleToNoteSaved() {
    yAngleToNoteSaved = 0;
  }

  public double fromCameraToTarget(double pitch, double yaw) {
    double alpha = Math.toRadians(-pitch + cameraPitchOffset);
    double yawForCalculation = Math.toRadians(Math.abs(yaw));

    double targetHorizontalDistance = heightOfCamera/Math.tan(alpha);

    return targetHorizontalDistance/Math.cos(yawForCalculation);
  }

  public double angleToTurnToNote(double yaw, double distance) {
    double alpha = Math.toRadians(180 - yaw);
    double centerOfRobotToTarget = Math.pow(centerOfRobotToCamera, 2) + Math.pow(distance, 2) - 
    2*centerOfRobotToCamera*distance*Math.cos(alpha);
    double beta = Math.toDegrees(Math.asin((Math.sin(alpha)*distance)/centerOfRobotToTarget));
    return beta;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
