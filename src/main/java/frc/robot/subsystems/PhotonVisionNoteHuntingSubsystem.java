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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
