// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GPMConstants.Arm;
import frc.robot.Constants.GPMConstants.Intake;
import frc.robot.Constants.GPMConstants.Intake.IntakePIDConstants;
import frc.robot.Constants.GPMConstants.Arm.ArmMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Arm.ArmPIDConstants;
import frc.robot.Constants.GPMConstants.Shooter.ShooterMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Shooter.ShooterPIDConstants;

@SuppressWarnings({ "removal" })

public class GPMSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */

  // === INTAKE ===

  // 775 connected to TalonSRX
  private WPI_TalonSRX intakeMotor; //TalonSRX
  
  // === SHOOTER ====

  // NEO motors connected to Spark Max
  private CANSparkMax shooterMotorLeft;
  private CANSparkMax shooterMotorRight;
  private CANSparkMax shooterMotorLeader;

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
  private CANSparkMax armMotorLeader;


  private SparkPIDController armPIDControllerLeft;
  private SparkPIDController armPIDControllerRight;

  // REV Thru-bore encoder connected to Spark Max, which has NEO
  // connected as well; so using Absolute Encoder expansion board
  private  SparkRelativeEncoder armEncoder;

  private static WPI_Pigeon2 armImu; // We will use downcasting to set this - it will point to methods either in NavX
  // or Pigeon subsystems
  private double currentArmIMU;
  

  //private SparkLimitSwitch noteSensor;  //limit switch

  public GPMSubsystem() {

    // ==========================
    // === ARM IMU initialization
    // ==========================
    armImu = new WPI_Pigeon2(Constants.IMUConstants.PIGEON2_CAN_ID);
    

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

     motor.restoreFactoryDefaults();

    // armMotorLeft.setSmartCurrentLimit(0);

    motor.setIdleMode(IdleMode.kBrake);

    if (c.getArmMotorFollower()) {
      motor.follow(motorToFollow);
    } else {
      armMotorLeader = motor;
    }

    // PID Controller setup

    // TODO: check if only one of these needs to be set
    p.setFeedbackDevice(armEncoder);

    // set arm PID coefficients - LIFT
    p.setP(ArmPIDConstants.kP);
    p.setI(ArmPIDConstants.kI);
    p.setD(ArmPIDConstants.kD);
    p.setIZone(ArmPIDConstants.Izone);
    p.setFF(ArmPIDConstants.kF);
    // kMaxOutput = 1 ; range is -1, 1
    p.setOutputRange(-ArmPIDConstants.kMaxOutput, ArmPIDConstants.kMaxOutput);

    // kMaxOutput = 1 ; range is -1, 1
    // armPIDControllerRight.setOutputRange(-Constants.GPMConstants.ArmPIDConstants.kMaxOutput,
    // Constants.GPMConstants.ShooterPIDConstants.kMaxOu
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
      shooterMotorLeader = motor;
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

  public void zeroArmEncoder() {  // zero encoders on master mmotor controllers of the drivetrain

    armEncoder.setPosition(0);
  
    System.out.println("===== arm encoders are 0");
  }

  public void configureIntakeMotor(){

    intakeMotor.configFactoryDefault();
    intakeMotor.setSafetyEnabled(false);

    //Configure motor and controller
    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    intakeMotor.setSensorPhase(Intake.INTAKE_SENSOR_PHASE);
    intakeMotor.setInverted(Intake.INTAKE_INVERTED);
    intakeMotor.configNeutralDeadband(Intake.INTAKE_NEUTRAL_DEADBAND, Intake.INTAKE_TIMEOUT);

    //PID Configuration
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, Intake.INTAKE_TIMEOUT);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, Intake.INTAKE_TIMEOUT);

    intakeMotor.configPeakOutputForward(+1.0, Intake.INTAKE_TIMEOUT);
    intakeMotor.configPeakOutputReverse(-1.0, Intake.INTAKE_TIMEOUT);
    intakeMotor.configNominalOutputForward(0, Intake.INTAKE_TIMEOUT);
    intakeMotor.configNominalOutputReverse(0, Intake.INTAKE_TIMEOUT);

    /* FPID Gains */
    intakeMotor.selectProfileSlot(IntakePIDConstants.intakeSlot0, IntakePIDConstants.pidIntakeIdx);
    intakeMotor.config_kP(IntakePIDConstants.intakeSlot0, IntakePIDConstants.kP,
      Intake.INTAKE_TIMEOUT);
    intakeMotor.config_kI(IntakePIDConstants.intakeSlot0, IntakePIDConstants.kI,
      Intake.INTAKE_TIMEOUT);
    intakeMotor.config_kD(IntakePIDConstants.intakeSlot0, IntakePIDConstants.kD,
      Intake.INTAKE_TIMEOUT);
    intakeMotor.config_kF(IntakePIDConstants.intakeSlot0, IntakePIDConstants.kF,
      Intake.INTAKE_TIMEOUT);

    intakeMotor.config_IntegralZone(IntakePIDConstants.intakeSlot0, IntakePIDConstants.Izone,
      Intake.INTAKE_TIMEOUT);
    intakeMotor.configClosedLoopPeakOutput(IntakePIDConstants.intakeSlot0, IntakePIDConstants.PeakOutput,
      Intake.INTAKE_TIMEOUT);
    intakeMotor.configAllowableClosedloopError(IntakePIDConstants.intakeSlot0,
      IntakePIDConstants.DefaultAcceptableError,
      Intake.INTAKE_TIMEOUT);

    intakeMotor.configClosedLoopPeriod(IntakePIDConstants.intakeSlot0, IntakePIDConstants.closedLoopPeriod,
      Intake.INTAKE_TIMEOUT);

    intakeMotor.configMotionAcceleration(IntakePIDConstants.Acceleration,
      Intake.INTAKE_TIMEOUT);
    intakeMotor.configMotionCruiseVelocity(IntakePIDConstants.CruiseVelocity,
      Intake.INTAKE_TIMEOUT);
    intakeMotor.configMotionSCurveStrength(IntakePIDConstants.intakeSmoothing);

  }

  // ======== PIGEON METHODS

  public double getPitch() {
    // double[] ypr = new double[3];
    // pigeon2.getYawPitchRoll(ypr);
    // return ypr[1];

    // Front UP - positive Pitch
    return -armImu.getPitch();
  }

  // ======== ARM METHODS

 public double getArmEnc(SparkAbsoluteEncoder c) {
    return c.getPosition();
  }

  public void getCurrentArmIMU() {
    currentArmIMU = getPitch() - RobotContainer.imuSubsystem.getPitch();
  }
  
  public double getArmAngle() {
    return (armEncoder.getPosition() + Arm.gearRatioDifference)*Arm.armDegreeEncoderConversion;
  }

  public void setArmMotorAnglesSI(double angle) {
   armMotorLeader.getPIDController().setReference((Arm.armDegreeEncoderConversion*angle), ControlType.kPosition);
   System.out.println("========== Arm Motor Angle set to : " + angle);
  }

  public void stopArmPID(){
    armMotorLeader.getPIDController().setReference((0), ControlType.kVoltage);
  }
  
  // ======== SHOOTER METHODS

  public void runShooter(double speed) {
    shooterMotorLeader.getPIDController().setReference((speed), ControlType.kVelocity);
    System.out.println("========== Shooter Motor running at " + speed);
  }

  public void stopShooter() {
    shooterMotorLeader.getPIDController().setReference((0), ControlType.kVelocity);
    System.out.println("========== Shooter Motor stopped ");
  }

  // ============== INTAKE METHODS

  public void runIntake(double speed) {
    intakeMotor.set(speed);
    System.out.println("========== Intake Motor running at " + speed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void getNoteSensor() { //????
    //noteSensor.isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
