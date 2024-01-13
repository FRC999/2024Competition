// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class GPMSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */
  private PWMTalonSRX intakeMotor;
  private PWMTalonSRX leftArm;
  private PWMTalonSRX rightArm;

  public GPMSubsystem() {
    intakeMotor = new PWMTalonSRX;
    leftArm = new PWMTalonSRX;
    rightArm = new PWMTalonSRX;
        
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
