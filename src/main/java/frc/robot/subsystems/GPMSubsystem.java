// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private AnalogEncoder armEncoder;
  
  private WPI_VictorSPX shooterA;
  private WPI_VictorSPX shooterB;

  private DigitalInput noteSensor;

  public GPMSubsystem() {

    //create arm motor variables
    intakeMotor = new WPI_TalonSRX(Constants.GPMConstants.INTAKE_MOTOR_CHANNEL);
    leftArm = new WPI_TalonSRX(Constants.GPMConstants.LEFT_ARM_CHANNEL);
    rightArm = new WPI_TalonSRX(Constants.GPMConstants.RIGHT_ARM_CHANNEL);
    //create shooter motor variables
    shooterA = new WPI_VictorSPX(Constants.GPMConstants.SHOOTER_A_CHANNEL);
    shooterB = new WPI_VictorSPX(Constants.GPMConstants.SHOOTER_B_CHANNEL);

    //motor setups    
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    leftArm.setNeutralMode(NeutralMode.Brake);
    leftArm.configOpenloopRamp(0.25);
    rightArm.setNeutralMode(NeutralMode.Brake);
    rightArm.configOpenloopRamp(0.25);
  }

  //TODO : figure out .getAbsolutePosition error for type Encoder
  
  public double getArmEnc() {
    return this.armEncoder.getAbsolutePosition();
  }
  

  public void armToPos(double pos) {
    double Kp = -15.0;
    double error = pos - this.getArmEnc();
    double power = Kp * error;
    this.moveArm(power);
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

  public void shootGamePiece(double power) {
    this.shooterA.set(-power);
    this.shooterB.set(-power);
  }

  public void getNoteSensor() {
    this.noteSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
