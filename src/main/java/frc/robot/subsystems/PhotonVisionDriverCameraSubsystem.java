// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnabledSubsystems;

public class PhotonVisionDriverCameraSubsystem extends SubsystemBase {

  PhotonCamera camera;

  /** Creates a new PhotonVisionDriverCameraSubsystem. */
  public PhotonVisionDriverCameraSubsystem() {
      if (! EnabledSubsystems.driverCamera) { return; }

      camera = new PhotonCamera("Microsoft Lifecam"); // TODO: check camera name
      camera.setDriverMode(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
