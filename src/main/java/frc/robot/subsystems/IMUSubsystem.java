// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IMUSubsystem extends SubsystemBase {

  private static WPI_Pigeon2 imu;

  private double trajectoryAdjustmentIMU; // This is the value we need to adjust the IMU by after Trajectory
                                          // is completed

  /**
   * Creates a new IMUSubsystem.
   * This is an IMU Passthrough class, meaning, it provides a class for the
   * generic IMU instantiation.The "switch" implementation chooser below will
   * instantiate hardware-specific
   * implementation, and will map "imu" variable to that object.
   * That means all hardware-specific implementations must implement the same
   * methods, which is enforced via interface IMUInterface. That interface
   * contains all methods that must be
   * declared as "public" both in this class and in the hardware-specific classes.
   * All other methods
   * (if they're needed) should be private.
   */
  public IMUSubsystem() {

    /*
     * Depending on the IMU type specified in Constants, the variable "imu" will
     * point to the instance
     * of the class specific to the hardware you have, e.g. Pigeon2 or NavX
     * All such implementation classes must have public methods specified by the
     * IMUInterface
     */
    
      imu = new WPI_Pigeon2(Constants.IMUConstants.PIGEON2_CHANNEL);
    
    }

    imu.zeroGyroBiasNow(); // TODO : At the start of game, robot must be pointed towards opposite team's side
                   // (This is our zero value). We do not know which side red/blue is on

  }

  /**
   * Note that all IMU methods that take or return values should do so in SI units.
   */

  public double getPitch() {
    return imu.getPitch();
  }

  public double getRoll() {
    return imu.getRoll();
  }

  public double getYaw() {
    return imu.getYaw();
  }

  public Rotation2d getRotation2d() {
    return imu.getRotation2d();
  }

  public void zeroYaw() {
    imu.setYaw(0);
  }

  public void setYaw(double y) {
    imu.setYaw(y);
  }

  /**
   * This method is used when we want to "snap" the chassis to a trajectory start, meaning
   * assuming that the robot is at the starting point of the trajectory.
   * Here we remember starting Yaw before trajectory so it can be restored
   * back after trajectory
   * @param y - starting Yaw of the trajectory
   * @return - old value of the Yaw (we do not currently use it)
   */
  public void setYawForTrajectory(double y) {
    trajectoryAdjustmentIMU = RobotContainer.imuSubsystem.getYaw() - y;
    imu.setYaw(y);
  }

  /**
   * Once the trajectory is done, we want to readjust the Yaw considering the value that we "remember", so
   * the field-centric drive axis will not change. That may allow one to drive automated trajectories in teleop
   * without losing the Yaw direction.
   */
  public void restoreYawAfterTrajectory() {
    imu.setYaw(RobotContainer.imuSubsystem.getYaw() + trajectoryAdjustmentIMU);
  }

/*
  public double getTurnRate() {
    return imu.getTurnRate();
  }
*/

  public double getCompassHeading() {
    return imu.getCompassHeading();
  }

}
