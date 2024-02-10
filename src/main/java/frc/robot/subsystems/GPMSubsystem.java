// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class GPMSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */
  private WPI_TalonSRX intakeMotor; //TalonSRX
  
  private SparkAbsoluteEncoder shooterEncoderA;
  private SparkAbsoluteEncoder shooterEncoderB;

  private CANSparkMax shooterA; //NEOS
  private CANSparkMax shooterB;

  private SparkPIDController armPIDControllerLeft;
  private SparkPIDController armPIDControllerRight;

  private SparkPIDController shooterPIDControllerA;
  private SparkPIDController shooterPIDControllerB;

  private CANSparkMax leftArm;  //NEOS
  private CANSparkMax rightArm;

  private  SparkAbsoluteEncoder armEncoderLeft;
  private  SparkAbsoluteEncoder armEncoderRight;

  //private SparkLimitSwitch noteSensor;  //limit switch

  public GPMSubsystem() {

    //create intake motor variables
    intakeMotor = new WPI_TalonSRX(Constants.GPMConstants.SHOOTER_MOTOR_CAN_ID);
    
    //create arm motor variables
    leftArm = new CANSparkMax(Constants.GPMConstants.LEFT_ARM_CAN_ID, MotorType.kBrushless);
    rightArm = new CANSparkMax(Constants.GPMConstants.RIGHT_ARM_CAN_ID, MotorType.kBrushless);
  
    armPIDControllerLeft = leftArm.getPIDController();
    armPIDControllerRight = rightArm.getPIDController();

     //create shooter motor variables
    shooterA = new CANSparkMax(Constants.GPMConstants.SHOOTER_A_CAN_ID, MotorType.kBrushless);
    shooterB = new CANSparkMax(Constants.GPMConstants.SHOOTER_B_CAN_ID, MotorType.kBrushless);

    armEncoderLeft = leftArm.getAbsoluteEncoder(Type.kDutyCycle);
    armEncoderLeft = rightArm.getAbsoluteEncoder(Type.kDutyCycle);
  }

  public void configureArmMotors() {
   
    leftArm.restoreFactoryDefaults();
    rightArm.restoreFactoryDefaults();

    leftArm.setSmartCurrentLimit(0);
    rightArm.setSmartCurrentLimit(0);

    leftArm.setIdleMode(IdleMode.kBrake);
    //leftArm.configOpenloopRamp(0.25);
    rightArm.setIdleMode(IdleMode.kBrake);
    //rightArm.configOpenloopRamp(0.25);

    leftArm.setInverted(true); 
    rightArm.setInverted(false); 
    
    leftArm.follow(rightArm);

    //PID Controllers 
    

    //set arm PID coefficients - LIFT
    armPIDControllerLeft.setP(Constants.GPMConstants.ArmPIDConstants.kP);
    armPIDControllerLeft.setI(Constants.GPMConstants.ArmPIDConstants.kI);
    armPIDControllerLeft.setD(Constants.GPMConstants.ArmPIDConstants.kD);
    armPIDControllerLeft.setIZone(Constants.GPMConstants.ArmPIDConstants.Izone);
    armPIDControllerLeft.setFF(Constants.GPMConstants.ArmPIDConstants.kF);
    //kMaxOutput = 1 ; range is -1, 1
    armPIDControllerLeft.setOutputRange(-Constants.GPMConstants.ArmPIDConstants.kMaxOutput, Constants.GPMConstants.ArmPIDConstants.kMaxOutput);

    //set arm PID coefficients - RIGHT
    armPIDControllerRight.setP(Constants.GPMConstants.ArmPIDConstants.kP);
    armPIDControllerRight.setI(Constants.GPMConstants.ArmPIDConstants.kI);
    armPIDControllerRight.setD(Constants.GPMConstants.ArmPIDConstants.kD);
    armPIDControllerRight.setIZone(Constants.GPMConstants.ArmPIDConstants.Izone);
    armPIDControllerRight.setFF(Constants.GPMConstants.ArmPIDConstants.kF);
    //kMaxOutput = 1 ; range is -1, 1
    armPIDControllerRight.setOutputRange(-Constants.GPMConstants.ArmPIDConstants.kMaxOutput, Constants.GPMConstants.ArmPIDConstants.kMaxOutput);
  }

  public void configureshooterMotors() {
    
    shooterA.restoreFactoryDefaults();
    shooterB.restoreFactoryDefaults();

    shooterA.setSmartCurrentLimit(0);
    shooterB.setSmartCurrentLimit(0);

    shooterA.setIdleMode(IdleMode.kBrake);
    shooterB.setIdleMode(IdleMode.kBrake);

    shooterB.follow(shooterA);

    //PID Controllers 
    shooterPIDControllerA.setFeedbackDevice(shooterEncoderA);
    shooterPIDControllerB.setFeedbackDevice(shooterEncoderB);

    //set arm PID coefficients - LIFT
    shooterPIDControllerA.setP(Constants.GPMConstants.ShooterPIDConstants.kP);
    shooterPIDControllerA.setI(Constants.GPMConstants.ShooterPIDConstants.kI);
    shooterPIDControllerA.setD(Constants.GPMConstants.ShooterPIDConstants.kD);
    shooterPIDControllerA.setIZone(Constants.GPMConstants.ShooterPIDConstants.Izone);
    shooterPIDControllerA.setFF(Constants.GPMConstants.ShooterPIDConstants.kF);
    //kMaxOutput = 1 ; range is -1, 1
    shooterPIDControllerA.setOutputRange(-Constants.GPMConstants.ShooterPIDConstants.kMaxOutput, Constants.GPMConstants.ShooterPIDConstants.kMaxOutput);

    //set arm PID coefficients - RIGHT
    shooterPIDControllerB.setP(Constants.GPMConstants.ShooterPIDConstants.kP);
    shooterPIDControllerB.setI(Constants.GPMConstants.ShooterPIDConstants.kI);
    shooterPIDControllerB.setD(Constants.GPMConstants.ShooterPIDConstants.kD);
    shooterPIDControllerB.setIZone(Constants.GPMConstants.ShooterPIDConstants.Izone);
    shooterPIDControllerB.setFF(Constants.GPMConstants.ShooterPIDConstants.kF);
    //kMaxOutput = 1 ; range is -1, 1
    //shooterPIDControllerB.setOutputRange(-Constants.GPMConstants.ShooterPIDConstants.kMaxOutput, Constants.GPMConstants.ShooterPIDConstants.kMaxOu
  }

  public void zeroArmEncoders() {  // zero encoders on master mmotor controllers of the drivetrain

    armEncoderLeft.setZeroOffset(0);
    armEncoderRight.setZeroOffset(0);
    System.out.println("===== arm encoders are 0");
  }

  public void configureIntakeMotor(){
    intakeMotor = new WPI_TalonSRX(Constants.GPMConstants.INTAKE_MOTOR_CAN_ID);
    intakeMotor.configFactoryDefault();
    intakeMotor.setSafetyEnabled(false);

    //PID Controllers
    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

 public double getArmEnc(SparkAbsoluteEncoder c) {
    return c.getPosition();
  }
  
  public double getShooterVelocity(AbsoluteEncoder c) {
    return c.getVelocity();
  }
  
  // public Rotation2d getArmAngle(CANSparkMax a) {
   
  // }
  

  public void runShooter(double speed) {
    shooterA.set(speed);
  }

  public void stopShooter() {
    
  }

  public void getNoteSensor() { //????
    //noteSensor.isPressed();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
