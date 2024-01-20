// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTableSubsystem extends SubsystemBase {
  private NetworkTableInstance ntInst;
  /** Creates a new NetworkTableSubsystem. */
  public NetworkTableSubsystem() {
    ntInst = NetworkTableInstance.getDefault(); //initializesNetworkTable
  }

  public double getHorizontalOffset() {
    return ntInst.getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getVerticalOffset() {
    return ntInst.getTable("limelight").getEntry("ty").getDouble(0);
  }

  public boolean targetSeen() {
    return ntInst.getTable("limelight").getEntry("tv").getDouble(0) == 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Target Spotted: ", targetSeen());
    SmartDashboard.putNumber("Horizontal Offset: ", getHorizontalOffset());
    SmartDashboard.putNumber("Vertical Offset: ", getVerticalOffset());
  }
}