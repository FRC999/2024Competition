// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.RobotContainer;
import frc.robot.SwerveRobotModule;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.SwerveChassis.*;

public class DriveSubsystem extends SubsystemBase {

  public SwerveRobotModule[] swerveMods;

  public DriveSubsystem() {

    swerveMods = new SwerveRobotModule[] {
        new SwerveRobotModule(0, SwerveModuleConstantsEnum.MOD0), // front left
        new SwerveRobotModule(1, SwerveModuleConstantsEnum.MOD1), // front right
        new SwerveRobotModule(2, SwerveModuleConstantsEnum.MOD2), // rear left
        new SwerveRobotModule(3, SwerveModuleConstantsEnum.MOD3) // rear right
    };

  }

  public double telemetryAngleEncoder(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoder();
  }

  public double telemetryAngleEncoderSI(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoderSI();
  }

  public double telemetryDriveEncoder(int modnumber) {
    return swerveMods[modnumber].telemetryDriveEncoder();
  }

  public void stopDriveMotor(int modnumber) {
    swerveMods[modnumber].DriveMotorApplyPower(0);
  }

  public void stopAngleMotor(int modnumber) {
    swerveMods[modnumber].AngleMotorApplyPower(0);
  }

  public void stopRobot() {
    drive(0, 0, 0, true);
  }

  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s, boolean fieldcentric) {
    SwerveModuleState[] swerveModuleStates;

    if (fieldcentric) { // field-centric swerve
      swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xVelocity_m_per_s,
              yVelocity_m_per_s,
              omega_rad_per_s,
              Rotation2d.fromDegrees(RobotContainer.imuSubsystem.getYaw())));
    } else { // robot-centric swerve; does not use IMU
      swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(
              xVelocity_m_per_s,
              yVelocity_m_per_s,
              omega_rad_per_s));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.MAX_VELOCITY);

    for (SwerveRobotModule mod : swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]);
    }

  }

  public void setDesiredStatesCalibration(SwerveModuleState[] swerveModuleStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.MAX_VELOCITY);

    for (SwerveRobotModule mod : swerveMods) {
      mod.setDesiredStateCalibration(swerveModuleStates[mod.getModuleNumber()]);
    }

  }

  public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.MAX_VELOCITY);

    for (SwerveRobotModule mod : swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]);
    }

  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = swerveMods[i].getPosition();
    }
    return positions;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
