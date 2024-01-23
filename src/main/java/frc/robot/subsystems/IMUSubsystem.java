package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

@SuppressWarnings({ "removal" })

/*
  * Since IMU represents a separate hardware component, it's defined as a
 * subsystem
 * (via "extends SubsystemBase"). It means that you can add "periodic" code to
 * it.
 * We do not currently need that.
 */

public class IMUSubsystem extends SubsystemBase {

  private static WPI_Pigeon2 imu; // We will use downcasting to set this - it will point to methods either in NavX
  // or Pigeon subsystems

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

    imu = new WPI_Pigeon2(Constants.IMUConstants.PIGEON2_CAN_ID);
    
    //I am calling this because for some reason zeroYaw gives error for 
    //  "The method zeroYaw() is undefined for the type WPI_Pigeon2"
    //imu.zeroYaw();
    imu.setYaw(0);

  }

  /**
   * Note that all IMU methods that take or return values should do so in SI
   * units.
   */

  public double getPitch() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // return ypr[1];

    // Front UP - positive Pitch
    return -imu.getPitch();
  }

  /**
   * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
   * axis that goes straight forward)
   * 
   * @return
   */
  public double getRoll() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // return ypr[2];

    // Left UP - positive Roll
    return imu.getRoll();
  }

  /**
   * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
   * robot is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getYaw() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // System.out.println(ypr[0]);
    // return ypr[0];

    return imu.getYaw(); // With Pigeon2 this method returns values in degrees
  }

  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  /**
   * Zeroes the yaw of the robot
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double previousYaw = getYaw();
    imu.setYaw(0);
    return previousYaw;
  }

  public double setYaw(double y) {
    double previousYaw = getYaw();
    imu.setYaw(y);
    return previousYaw;
  }

  public Rotation2d getRotation2d() {
    return imu.getRotation2d();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    imu.setYaw(0);
    System.out.println("Yaw and Fused Heading set");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180 in Rotation2d format
   */
  public Rotation2d getHeading() {
    return imu.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -imu.getRate();
  }

    /**
   * This method is used when we want to "snap" the chassis to a trajectory start, meaning
   * assuming that the robot is at the starting point of the trajectory.
   * Here we remember starting Yaw before trajectory so it can be restored
   * back after trajectory
   * @param y - starting Yaw of the trajectory
   * @return - old value of the Yaw (we do not currently use it)
   */
  public double setYawForTrajectory(double y) {
    trajectoryAdjustmentIMU = RobotContainer.imuSubsystem.getYaw() - y;

    // alex test
    //System.out.println("----  Trajectory AdjustmentIMU: "+ trajectoryAdjustmentIMU + " Real: "+ RobotContainer.imuSubsystem.getYaw()) ;

    return setYaw(y);  // our own setYaw that returns old angle
  }

  /**
   * Once the trajectory is done, we want to readjust the Yaw considering the value that we "remember", so
   * the field-centric drive axis will not change. That may allow one to drive automated trajectories in teleop
   * without losing the Yaw direction.
   */
  public void restoreYawAfterTrajectory() {
    //System.out.println("Restoring original IMU after trajectory "+ (RobotContainer.imuSubsystem.getYaw() + trajectoryAdjustmentIMU));
    imu.setYaw(RobotContainer.imuSubsystem.getYaw() + trajectoryAdjustmentIMU);
  }

}