// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GPMConstants.Intake;
import frc.robot.Constants.GPMConstants.Arm.ArmMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Arm.ArmPIDConstants;
import frc.robot.Constants.GPMConstants.Shooter.ShooterMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Shooter.ShooterPIDConstants;



public class GPMSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */

  // === INTAKE ===

  // 775 connected to TalonSRX
  private WPI_TalonSRX intakeMotor; //TalonSRX
  
  // === SHOOTER ====

  // NEO motors connected to Spark Max
  private CANSparkMax shooterMotorLeft;
  private CANSparkMax shooterMotorRight;

  // Built-in NEO encoders
  // Will be used with Velocity PID
  private RelativeEncoder shooterEncoderLeft;
  private RelativeEncoder shooterEncoderRight;

  // Necessary for hardware PID with Spark Max
  private SparkPIDController shooterPIDControllerLeft;
  private SparkPIDController shooterPIDControllerRight;

  // === ARM ====

  // NEO motors connected to Spark Max
  private CANSparkMax armMotorLeft; 
  private CANSparkMax armMotorRight;

  private SparkPIDController armPIDControllerLeft;
  private SparkPIDController armPIDControllerRight;

  // REV Thru-bore encoder connected to Spark Max, which has NEO
  // connected as well; so using Absolute Encoder expansion board
  private  SparkAbsoluteEncoder armEncoderLeft;
  private  SparkAbsoluteEncoder armEncoderRight;

  //private SparkLimitSwitch noteSensor;  //limit switch

  public GPMSubsystem() {

    // ==========================
    // === INTAKE initiatization
    // ==========================

    intakeMotor = new WPI_TalonSRX(Intake.INTAKE_MOTOR_CAN_ID);

    configureIntakeMotor();
    
    // ==========================
    // === ARM initialization
    // ==========================

    armMotorLeft = new CANSparkMax(ArmMotorConstantsEnum.LEFTMOTOR.getArmMotorID(), MotorType.kBrushless);
    armMotorRight = new CANSparkMax(ArmMotorConstantsEnum.RIGHTMOTOR.getArmMotorID(), MotorType.kBrushless);

    //TODO: We will probably have only one Thru-bore encoder, which is sufficient for us; revise the code as needed
    armEncoderLeft = armMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
    armEncoderLeft = armMotorRight.getAbsoluteEncoder(Type.kDutyCycle);

    armPIDControllerLeft = armMotorLeft.getPIDController();
    armPIDControllerRight = armMotorRight.getPIDController();

    // Main Motor; should not follow the other motor
    configureArmMotors(armMotorLeft, armPIDControllerLeft, ArmMotorConstantsEnum.LEFTMOTOR, null);
    // Follower Motor
    configureArmMotors(armMotorRight, armPIDControllerRight, ArmMotorConstantsEnum.RIGHTMOTOR, armMotorLeft);

    // ==========================
    // === SHOOTER initialization
    // ==========================

    shooterMotorLeft = new CANSparkMax(ShooterMotorConstantsEnum.LEFTMOTOR.getShooterMotorID(), MotorType.kBrushless);
    shooterMotorRight = new CANSparkMax(ShooterMotorConstantsEnum.RIGHTMOTOR.getShooterMotorID(), MotorType.kBrushless);

    shooterEncoderLeft = shooterMotorLeft.getEncoder();
    shooterEncoderRight = shooterMotorRight.getEncoder();

    // Main Motor; should not follow the other motor
    configureshooterMotors(shooterMotorLeft, shooterPIDControllerLeft, ShooterMotorConstantsEnum.LEFTMOTOR, null);
    // Follower Motor
    configureshooterMotors(shooterMotorRight, shooterPIDControllerRight, ShooterMotorConstantsEnum.RIGHTMOTOR, shooterMotorLeft);

  }

  /**
   * Configure Arm motors with a main and a follower
   * @param motor - motor object
   * @param p - PID controller object
   * @param c - motor constants
   * @param motorToFollow - motor to follow if this is a follower
   */
  public void configureArmMotors(CANSparkMax motor, SparkPIDController p, ArmMotorConstantsEnum c,
      CANSparkMax motorToFollow) {
   
    motor.restoreFactoryDefaults();

    motor.clearFaults();

    // motor.setSmartCurrentLimit(0);

    motor.setIdleMode(IdleMode.kBrake);

    armMotorLeft.setInverted(true); 
    armMotorRight.setInverted(false); 
    
    if (c.getArmMotorFollower()) {
      motor.follow(motorToFollow);
    } else {

    }

    //PID Controllers 
    
    p.setFeedbackDevice(shooterEncoderLeft);

    // set arm PID coefficients - LIFT
    p.setP(ArmPIDConstants.kP);
    p.setI(ArmPIDConstants.kI);
    p.setD(ArmPIDConstants.kD);
    p.setIZone(ArmPIDConstants.Izone);
    p.setFF(ArmPIDConstants.kF);
    // kMaxOutput = 1 ; range is -1, 1
    p.setOutputRange(-ArmPIDConstants.kMaxOutput, ArmPIDConstants.kMaxOutput);

    //kMaxOutput = 1 ; range is -1, 1
    armPIDControllerRight.setOutputRange(-ArmPIDConstants.kMaxOutput, ArmPIDConstants.kMaxOutput);
  }

  /**
   * Configure Shooter motors with a main and a follower
   * @param motor - motor object
   * @param p - PID controller object
   * @param c - motor constants
   * @param motorToFollow - motor to follow if this is a follower
   */
  public void configureshooterMotors(CANSparkMax motor, SparkPIDController p, ShooterMotorConstantsEnum c,
      CANSparkMax motorToFollow) {

    motor.restoreFactoryDefaults();

    // shooterMotorLeft.setSmartCurrentLimit(0);

    motor.setIdleMode(IdleMode.kBrake);

    if (c.getShooterMotorFollower()) {
      motor.follow(motorToFollow);
    } else {

    }

    // PID Controller setup

    // TODO: check if only one of these needs to be set
    p.setFeedbackDevice(shooterEncoderLeft);

    // set arm PID coefficients - LIFT
    p.setP(ShooterPIDConstants.kP);
    p.setI(ShooterPIDConstants.kI);
    p.setD(ShooterPIDConstants.kD);
    p.setIZone(ShooterPIDConstants.Izone);
    p.setFF(ShooterPIDConstants.kF);
    // kMaxOutput = 1 ; range is -1, 1
    p.setOutputRange(-ShooterPIDConstants.kMaxOutput, ShooterPIDConstants.kMaxOutput);

    // kMaxOutput = 1 ; range is -1, 1
    // shooterPIDControllerB.setOutputRange(-Constants.GPMConstants.ShooterPIDConstants.kMaxOutput,
    // Constants.GPMConstants.ShooterPIDConstants.kMaxOu
  }

  public void zeroArmEncoders() {  // zero encoders on master mmotor controllers of the drivetrain

    armEncoderLeft.setZeroOffset(0);
    armEncoderRight.setZeroOffset(0);
    System.out.println("===== arm encoders are 0");
  }

  public void configureIntakeMotor(){

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
    shooterMotorLeft.set(speed);
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
