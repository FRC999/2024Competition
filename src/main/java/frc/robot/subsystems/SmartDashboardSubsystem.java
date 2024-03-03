// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {

  }

  /**
   * If you turn a wheel counnter-clockwise the angle value and SI value should increase
   */
  public void updateOdometryTelemetry() {
     for (int i =0; i<4; i++){
       SmartDashboard.putNumber("S"+i+" Angle Encoder", RobotContainer.driveSubsystem.telemetryAngleEncoder(i));
       SmartDashboard.putNumber("S"+i+" Angle Encoder SI", RobotContainer.driveSubsystem.telemetryAngleEncoderSI(i));
       SmartDashboard.putNumber("S"+i+" Angle Encoder SI Abs", RobotContainer.driveSubsystem.telemetryAngleEncoderSIAbs(i));
       SmartDashboard.putNumber("S"+i+" Drive Encoder", RobotContainer.driveSubsystem.telemetryDriveEncoder(i));
       //SmartDashboard.putNumber("S"+i+" Drive Encoder", RobotContainer.driveSubsystem.telemetryDriveEncoderSI(i));
       SmartDashboard.putNumber("S"+i+" CANCoder", RobotContainer.driveSubsystem.telemetryCANCoderSI(i));
      
    }
    
  }

  public void updateVisionTelemetryLL() {
    SmartDashboard.putBoolean("LL AT present", RobotContainer.llVisionSubsystem.isApriltagVisible());
    Pose2d llPose = RobotContainer.llVisionSubsystem.getRobotFieldPoseLL();
    if (llPose == null) {
      SmartDashboard.putString("LL RobotPose2d", "null");
      SmartDashboard.putNumber("LL Distance BlueS", Double.NaN);
      SmartDashboard.putNumber("LL AngleD BlueS", Double.NaN);
    } else {
      SmartDashboard.putString("LL RobotPose2d", llPose.toString());
      SmartDashboard.putNumber("LL Distance BlueS", RobotContainer.llVisionSubsystem.getDistanceToBlueSpeaker(llPose));
      SmartDashboard.putNumber("LL AngleD BlueS", RobotContainer.llVisionSubsystem.getAngleToBlueSpeaker(llPose).getDegrees());
    }
  }

  public void updateVisionTelemetryPV() {
    SmartDashboard.putBoolean("PV AT present", RobotContainer.photonVisionSubsystem.isApriltagVisible());
    SmartDashboard.putString("PV RobotPose2d", RobotContainer.photonVisionSubsystem.getRobotFieldPosePV().toString());
    SmartDashboard.putNumber("PV Distance BlueS", RobotContainer.photonVisionSubsystem.getDistanceToBlueSpeaker(RobotContainer.photonVisionSubsystem.getRobotFieldPosePV()));
    SmartDashboard.putNumber("PV AngleD BlueS", RobotContainer.photonVisionSubsystem.getAngleToBlueSpeaker(RobotContainer.photonVisionSubsystem.getRobotFieldPosePV()).getDegrees());
  }

  // Note tracking
  public void updateVisionDetectorTelemetry() {
    SmartDashboard.putBoolean("Note Present", RobotContainer.llDetectorSubsystem.isNoteVisible());
    SmartDashboard.putNumber("Note THorizontal", RobotContainer.llDetectorSubsystem.getHorizontalSide());
    SmartDashboard.putNumber("Note TX", RobotContainer.llDetectorSubsystem.getHorizontalOffset());
    SmartDashboard.putNumber("Note TY", RobotContainer.llDetectorSubsystem.getVerticalOffset());
  }

  public void updateIMUTelemetry() {
    SmartDashboard.putNumber("IMU Yaw", RobotContainer.imuSubsystem.getYaw());
  }

  public void updateArmTelemetry() {
    SmartDashboard.putNumber("ArmLeftEncoder", RobotContainer.armSubsystem.getArmEncoderLeft());
    SmartDashboard.putNumber("ArmRightEncoder", RobotContainer.armSubsystem.getArmEncoderRight());
    SmartDashboard.putNumber("ArmLeadingEncoder", RobotContainer.armSubsystem.getArmEncoderLeader());
    SmartDashboard.putNumber("ArmRightVelocity", RobotContainer.armSubsystem.getRightArmMotorVelocity());
    SmartDashboard.putNumber("ArmLeftVelocity", RobotContainer.armSubsystem.getLeftArmMotorVelocity());
    SmartDashboard.putNumber("ArmAngle", RobotContainer.armSubsystem.getArmAngleSI());
    SmartDashboard.putNumber("ArmIMUPitch", RobotContainer.armSubsystem.getArmIMUPitch());
    SmartDashboard.putNumber("IMU Pitch", RobotContainer.imuSubsystem.getPitch());
  }

  public void updateShooterTelemetry() {
    SmartDashboard.putNumber("ShooterLeftEncoder", RobotContainer.shooterSubsystem.getLeftShooterMotorEncoder());
    SmartDashboard.putNumber("ShooterRightEncoder", RobotContainer.shooterSubsystem.getRightShooterMotorEncoder());
    SmartDashboard.putNumber("ShooterLeftVelocity", RobotContainer.shooterSubsystem.getLeftShooterMotorVelocity());
    SmartDashboard.putNumber("ShooterRightVelocity", RobotContainer.shooterSubsystem.getRightShooterMotorVelocity());
  }

  public void updateIntakeTelemetry() {
    if (Constants.GPMConstants.Intake.NOTE_SENSOR_PRESENT) {
      SmartDashboard.putBoolean("Intake sensor : ", RobotContainer.intakeSubsystem.isNoteInIntake());
    }
    if (Constants.GPMConstants.Intake.INTAKE_DOWN_LIMIT_SWITCH_PRESENT) {
      SmartDashboard.putBoolean("Intake sensor : ", RobotContainer.intakeSubsystem.isIntakeDown());
    }
  }

  public void updateAllDisplays(){

    if (DebugTelemetrySubsystems.odometry) {
      updateOdometryTelemetry();
    }

    if (DebugTelemetrySubsystems.imu) {
      updateIMUTelemetry();
    }

    if (EnabledSubsystems.arm && DebugTelemetrySubsystems.arm) {
      updateArmTelemetry();
    }

    if (EnabledSubsystems.shooter && DebugTelemetrySubsystems.shooter) {
      updateShooterTelemetry();
    }

    if (EnabledSubsystems.intake && DebugTelemetrySubsystems.intake) {
      updateIntakeTelemetry();
    }

    // Test vision
    //updateVisionTelemetryLL();
    updateVisionDetectorTelemetry();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
