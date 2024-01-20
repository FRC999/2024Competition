// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class GPMSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */
  private WPI_TalonSRX intakeMotor; //TalonSRX
  
  private CANSparkMax leftArm;  //NEOS
  private CANSparkMax rightArm;
  
  private RelativeEncoder armEncoder;
  
  private CANSparkMax shooterA; //NEOS
  private CANSparkMax shooterB;

  private SparkLimitSwitch noteSensor;  //limit switch

  public GPMSubsystem() {

    //create intake motor variables
    intakeMotor = new WPI_TalonSRX(Constants.GPMConstants.INTAKE_MOTOR_CAN_ID);
    
    //create arm motor variables
    leftArm = new CANSparkMax(Constants.GPMConstants.LEFT_ARM_CAN_ID, MotorType.kBrushless);
    rightArm = new CANSparkMax(Constants.GPMConstants.RIGHT_ARM_CAN_ID, MotorType.kBrushless);
    
    //create shooter motor variables
    shooterA = new CANSparkMax(Constants.GPMConstants.SHOOTER_A_CAN_ID, MotorType.kBrushless);
    shooterB = new CANSparkMax(Constants.GPMConstants.SHOOTER_B_CAN_ID, MotorType.kBrushless);


    //motor setups    
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    leftArm.setIdleMode(IdleMode.kBrake);
    //leftArm.configOpenloopRamp(0.25);
    rightArm.setIdleMode(IdleMode.kBrake);
    //rightArm.configOpenloopRamp(0.25);

    leftArm.setInverted(true); 
    rightArm.setInverted(false); 
    leftArm.follow(rightArm);

    shooterB.follow(shooterA);
  }
  
  public double getArmEnc() {
    return this.armEncoder.getPosition();
  }
  

  public void armToPos(double pos) {
    double Kp = -0.57;
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

    this.rightArm.set(power); //their motors are pointing in opposite directions

  }

  public void intakeGamePiece(double power) {
    this.intakeMotor.set(power);
  }

  public void shootGamePiece(double power) {
    this.shooterA.set(-power);
  }

  public void getNoteSensor() {
    this.noteSensor.isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
