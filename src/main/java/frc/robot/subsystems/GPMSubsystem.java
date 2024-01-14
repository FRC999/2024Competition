// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants;




public class GPMSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */
  private WPI_TalonSRX intakeMotor;
  private WPI_TalonSRX leftArm;
  private WPI_TalonSRX rightArm;
  private Encoder armEncoder;

  public GPMSubsystem() {
    
    //TODO : fix errors on .configOpenloopRamp and .setNeutralMode
    //intakeMotor.setNeutralMode(NeutralMode.Brake);
    //leftArm.setNeutralMode(NeutralMode.Brake);
    //leftArm.configOpenloopRamp(0.25);
    //rightArm.setNeutralMode(NeutralMode.Brake);
    //rightArm.configOpenloopRamp(0.25);
    intakeMotor = new WPI_TalonSRX(Constants.GPMConstants.INTAKE_MOTOR_CHANNEL);
    leftArm = new WPI_TalonSRX(Constants.GPMConstants.LEFT_ARM_CHANNEL);
    rightArm = new WPI_TalonSRX(Constants.GPMConstants.RIGHT_ARM_CHANNEL);
        
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    leftArm.setNeutralMode(NeutralMode.Brake);
    leftArm.configOpenloopRamp(0.25);
    rightArm.setNeutralMode(NeutralMode.Brake);
    rightArm.configOpenloopRamp(0.25);
  }

  //TODO : figure out .getAbsolutePosition error for type Encoder
  /* 
  public double getArmEnc() {
    return this.armEncoder.getAbsolutePosition();
  }
  */

  public void armToPos(double pos) {
    double Kp = -15.0;
    //double error = pos - this.armEncoder.getAbsolutePosition();
    //double power = Kp * error;
    //this.moveArm(power);
  }

  public void moveArm(double power) {
    double maxPower = 0.5;

    //stop from making too much torque
    if (power > maxPower) {
      power = maxPower;
    } else if (power < -maxPower) {
      power = -maxPower;
    }

    this.leftArm.set(-power); //their motors are pointing in opposite directions
    this.rightArm.set(-power);

  }

  public void intakeGamePiece(double power) {
    this.intakeMotor.set(power);
  }

  //TODO : how/where did they create this shooter object
  /* 
  public void shootGamePiece(double power) {
    this.shooter.set
  }
` */






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
