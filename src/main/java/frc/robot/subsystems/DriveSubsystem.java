// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.SwerveChassis.*;

public class DriveSubsystem extends SubsystemBase {

  public SwerveRobotModule[] swerveMods;

  public SwerveDriveOdometry swerveOdometry;
  public SwerveDrivePoseEstimator swervePoseEstimator;


  

  public DriveSubsystem() {

    swerveMods = new SwerveRobotModule[] {
        new SwerveRobotModule(0, SwerveModuleConstantsEnum.MOD0), // front left
        new SwerveRobotModule(1, SwerveModuleConstantsEnum.MOD1), // front right
        new SwerveRobotModule(2, SwerveModuleConstantsEnum.MOD2), // rear left
        new SwerveRobotModule(3, SwerveModuleConstantsEnum.MOD3) // rear right
    };

    // When the robot is turned on, both the IMU and drive encoders will be set to 0.
    // So, the initial odometry X,Y,Angle will be set to 0,0,0
    // This may need to be updated later either by th auto-driviing routines, or by camera inputs based on the AprilTags
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveChassis.SWERVE_KINEMATICS, RobotContainer.imuSubsystem.getYawRotation2d(), getPositions());

    // This object will help tracking Swerve pose changes based on the odometry
    // So it will likely be used only for telemetry
    swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveChassis.SWERVE_KINEMATICS, RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), new Pose2d());

  }

  public double telemetryAngleEncoder(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoder();
  }

  public double telemetryAngleEncoderSI(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoderSI();
  }

  public double telemetryAngleEncoderSIAbs(int modnumber) {
    return swerveMods[modnumber].telemetryAngleEncoderSIAbs();
  }

  public double telemetryDriveEncoder(int modnumber) {
    return swerveMods[modnumber].telemetryDriveEncoder();
  }

  public double telemetryCANCoderSI(int modnumber) {
    return swerveMods[modnumber].telemetryCANCoderSI();
  }

  public double telemetryCANCoder(int modnumber) {
    return swerveMods[modnumber].telemetryCANCoder();
  }

  public void stopDriveMotor(int modnumber) {
    swerveMods[modnumber].driveMotorApplyPower(0);
  }

  public void stopAngleMotor(int modnumber) {
    swerveMods[modnumber].angleMotorApplyPower(0);
  }

  public void stopRobot() {
    drive(0, 0, 0, true);
  }

  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s, boolean fieldcentric) {
    SwerveModuleState[] swerveModuleStates;

    //alex text
    //System.out.println("O:"+omega_rad_per_s);

    if (fieldcentric) { // field-centric swerve
      if (RobotContainer.isAlianceRed && RobotContainer.isReversingControllerAndIMUForRed)  { // On red alliance reverse X and Y directions to preserve BLUE coordinate system
        xVelocity_m_per_s = -xVelocity_m_per_s;
        yVelocity_m_per_s = -yVelocity_m_per_s;
      }
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

   /**
   * Set odometry to a specified field-centric Pose2d
   * You may need to do so for the trajectory driving, if you want the robot to assume being at the
   * start of the trajectory.
   * Be aware that on-going odometry updates use IMU. So, your odometry yaw may change incorrectly
   * later if the current yaw is not reset properly on the IMU first.
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), pose);
  }

  /** 
   * Field Centric Pose of the chassis
   * We get it from odometry, rather than sensors. That means commands that use it must ensure that
   * odometry was properly updated.
  */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Print odometry values from the current state of the odometry object (ServeDriveOdometry).
   * It's used only for telemetry. Use the getPose() method to get the values returned.
   */
  public void odometryTelemetry() {
    //RobotContainer.myStringLog.append("Odometry: "+swerveOdometry.getPoseMeters());
    System.out.println("Odometry: "+swerveOdometry.getPoseMeters());
  }

  /**
   * This method was designed to print proposed update to the odometry, and can be used for testing
   * if you suspect the odometry values you're submitting are wrong. It may be usefult in
   * catching issues related to the units of measure conversion, or measure imprecission in conversion from
   * the encoder units to the SI units
   * @param r - rotation of the chassis in Rotation2d
   * @param s - array of positions for each individual swerve module
   */
  public void odometryCommandTelemetry(Rotation2d r, SwerveModulePosition[] s) {
    //RobotContainer.myStringLog.append("Odometry Command Rotation: "+r);
    System.out.println("Odometry Command Rotation: "+r);
    
    for(int i = 0; i < 4; i++)  {
      //RobotContainer.myStringLog.append("Odometry Command Module: "+ i + " " + s[i]);
      System.out.println("Odometry Command Module: "+ i + " " + s[i]);
    }
  
  }

  public void resetPoseEstimator(Pose2d pose) {
    swervePoseEstimator.resetPosition(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions(), pose);
  }

  public Pose2d getPoseEstimate() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * This method updates swerve odometry. Note that we do not update it in a periodic method
   * to reduce the CPU usage, since the odometry is only used for trajectory driving.
   * If you use odometry for other purposes, such as using vision to track game elements,
   * either update the odometry in periodic method, or update it from appropriate commands.
   * 
   * The telemetry will print the IMU and swerve positions we send to odometry along with
   * the same information obtained from telemetry after update.
   * It may help troubleshooting potential odometry update issues (e.g. units of
   * measure issues etc)
   * 
   * Please, note that use of telemetry may significantly increase CPU usage, and can ultimately result
   * in a packet loss. We recommend disabling excessive telemetry for the competition.
   */
  public void updateTrajectoryOdometry() {
    if (SwerveTelemetry.odometryTelemetryPrint) {
      //RobotContainer.myStringLog.append("PoseSupplier: "+getPose());
      System.out.println("PoseSupplier: "+getPose());
      Rotation2d r = RobotContainer.imuSubsystem.getYawRotation2d();
      SwerveModulePosition[] s = getPositions();
      odometryCommandTelemetry(r, s);
      swerveOdometry.update(r, s);
      odometryTelemetry();
    } else {
      swerveOdometry.update(RobotContainer.imuSubsystem.getYawRotation2d(), getPositions());
    }
  }

  /**
   * This method was designed solely for the test comand so you can see whether odometry was updated correctly.
   * It updates gyroAngle to 10 degrees, then reads it and prints right away. If the result is not 10 degrees,
   * something must be wrong.
   */
  public void testOdometryUpdates() {
    swerveOdometry.update(
      Rotation2d.fromDegrees(10),
      getPositions());
    odometryTelemetry();
  }

  /*This the start of the Chassis related methods that are related to the Autonomous!! */

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setDesiredStates(SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getRobotVelocity(); // robot-relative chassis speeds
  }

   // Used only for motor testing; run motor forward, 0.3 power
  public void testDriveMotorEncoderPhase(int modnumber){
    //swerveMods[modnumber].testDriveMotorApplyPower(0.3);
    swerveMods[modnumber].testDriveMotorApplyPower(RobotContainer.driveStick.getRawAxis(3));
  }

  // Used only for motor testing; run motor forward, 0.3 power
  public void testAngleMotorEncoderPhase(int modnumber) {
    swerveMods[modnumber].testAngleMotorApplyPower(0.3);
  }


  /**
   * PathPlanner 2024-specific new methods
   */

  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative
    // speeds,
    // but not the reverse. However, because this transform is a simple rotation,
    // negating the
    // angle
    // given as the robot angle reverses the direction of rotation, and the
    // conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        SwerveChassis.SWERVE_KINEMATICS.toChassisSpeeds(getStates()), RobotContainer.imuSubsystem.getYawRotation2d());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return SwerveChassis.SWERVE_KINEMATICS.toChassisSpeeds(getStates());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i=0; i<4; i++) {
      states[i] = swerveMods[i].getState();
    }
    return states;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
