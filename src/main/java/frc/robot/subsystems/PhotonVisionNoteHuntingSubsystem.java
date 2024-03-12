// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnabledSubsystems;

public class PhotonVisionNoteHuntingSubsystem extends SubsystemBase {

  PhotonCamera camera;
  private boolean cameraConnected;

  /** Creates a new PhotonVisionNoteHuntingSubsystem. */
  public PhotonVisionNoteHuntingSubsystem(String cameraName) {
          if (! EnabledSubsystems.noteHuntingCamera) { return; }

      camera = new PhotonCamera(cameraName); // TODO: check camera name

      cameraConnected = camera.isConnected();

      System.out.println("C:"+cameraConnected);
  }

  public boolean isNoteDetected() {
    var result = camera.getLatestResult();
    return result.hasTargets();
  }

  public double xAngleToNote() {
    var result = camera.getLatestResult();
    if (!result.hasTargets()) { // if I do not see notes return 0 angle
      return 0;
    }
    PhotonTrackedTarget target = result.getBestTarget();
    return -target.getYaw(); // getYaw here returns positive right

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
