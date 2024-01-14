// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class GPMSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */
  private WPI_TalonSRX intakeMotor;
  private WPI_TalonSRX leftArm;
  private WPI_TalonSRX rightArm;

  public GPMSubsystem() {
    intakeMotor = new WPI_TalonSRX(Constants.GPMConstants.INTAKE_MOTOR_CHANNEL);
    leftArm = new WPI_TalonSRX(Constants.GPMConstants.LEFT_ARM_CHANNEL);
    rightArm = new WPI_TalonSRX(Constants.GPMConstants.RIGHT_ARM_CHANNEL);
        
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    leftArm.setNeutralMode(NeutralMode.Brake);
    leftArm.configOpenloopRamp(0.25);
    rightArm.setNeutralMode(NeutralMode.Brake);
    rightArm.configOpenloopRamp(0.25);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
