// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.Arm;

import frc.robot.Constants.GPMConstants.Arm.ArmMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Arm.ArmPIDConstants;


@SuppressWarnings({ "removal" })

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new GPMSubsystem. */
  
  // === ARM ====

  // NEO motors connected to Spark Max
  private CANSparkMax armMotorLeft; 
  private CANSparkMax armMotorRight;
  private CANSparkMax armMotorLeader;


  private SparkPIDController armPIDControllerLeft;
  private SparkPIDController armPIDControllerRight;

  // We wii use built-in NEO encoders for now
  // They're relative, but we can calibrate them based on second Pigeon on the arm
  private  RelativeEncoder armEncoderLeft;
  private  RelativeEncoder armEncoderRight;
  private  RelativeEncoder armEncoderLeading;

  private WPI_Pigeon2 armImu;
  private double armEncoderZero;

  //private SparkLimitSwitch noteSensor;  //limit switch

  public ArmSubsystem() {

    // Check if need to initialize arm
    if (! EnabledSubsystems.arm) { return; }

    // ==========================
    // === ARM initialization
    // ==========================

    armMotorLeft = new CANSparkMax(ArmMotorConstantsEnum.LEFTMOTOR.getArmMotorID(), MotorType.kBrushless);
    armMotorRight = new CANSparkMax(ArmMotorConstantsEnum.RIGHTMOTOR.getArmMotorID(), MotorType.kBrushless);

    // TODO: We will probably have only one Thru-bore encoder, which is sufficient
    // for us; revise the code as needed

    armPIDControllerLeft = armMotorLeft.getPIDController();
    armPIDControllerRight = armMotorRight.getPIDController();

    // Set Arm encoders
    armEncoderLeft = armMotorLeft.getEncoder();
    armEncoderRight = armMotorRight.getEncoder();

    // Main Motor; should not follow the other motor
    configureArmMotors(armMotorLeft, armPIDControllerLeft, ArmMotorConstantsEnum.LEFTMOTOR, null);
    // Follower Motor
    configureArmMotors(armMotorRight, armPIDControllerRight, ArmMotorConstantsEnum.RIGHTMOTOR, armMotorLeft);

    // ==========================
    // === ARM IMU initialization
    // ==========================
    // This IMU should be attached FLAT to the ARM, with X pointing straight forward.
    // The IMU angle will allow us to calibrate NEO encoders rotating the arm.
    armImu = new WPI_Pigeon2(Arm.PIGEON2_ARM_CAN_ID);
    calibrateArmEncoderToPitch();

  }

  /**
   * Configure Arm motors with a main and a follower
   * 
   * @param motor         - motor object
   * @param p             - PID controller object
   * @param c             - motor constants
   * @param motorToFollow - motor to follow if this is a follower
   */
  private void configureArmMotors(CANSparkMax motor, SparkPIDController p, ArmMotorConstantsEnum c,
      CANSparkMax motorToFollow) {

    motor.restoreFactoryDefaults();
    motor.clearFaults();
    motor.setInverted(c.getArmMotorInverted());

    motor.setIdleMode(IdleMode.kBrake);

    if (c.getArmMotorFollower()) {
      motor.follow(motorToFollow);
    } else {
      armMotorLeader = motor;
      armEncoderLeading = motor.getEncoder();
    }

    // --- PID Setup
    // set the PID sensor to motor encoder for hardware PID
    p.setFeedbackDevice(motor.getEncoder());

    // set arm PID coefficients - LIFT
    p.setP(ArmPIDConstants.kP);
    p.setI(ArmPIDConstants.kI);
    p.setD(ArmPIDConstants.kD);
    p.setIZone(ArmPIDConstants.Izone);
    p.setFF(ArmPIDConstants.kF);
    // kMaxOutput = 1 ; range is -1, 1
    p.setOutputRange(-ArmPIDConstants.kMaxOutput, ArmPIDConstants.kMaxOutput);

  }

  // ======== ARM METHODS

  /**
   * The NEO built-in encoder cannot be reset.
   * So, instead we remember its "Zero position" using the IMU
   */
  private void calibrateArmEncoderToPitch() {
    armEncoderZero = getArmIMUPitch() - 
      ((Arm.USE_PAN_IMU_FOR_CORRECTION)?RobotContainer.imuSubsystem.getPitch():0 )
          *Arm.ARM_ENCODER_CHANGE_PER_DEGREE ;
  }

  /**
   * Get Arm IMU pitch in degrees.
   * IMU is pointed to the FRONT of the robot with the X and left with Y
   * Positive Pitch angle increases when arm is going from front towards the back.
   * @return
   */
  public double getArmIMUPitch() {
    return -armImu.getPitch();
  }

  /**
   * Get current angle of the arm.
   * Positive Pitch angle increases when arm is going from front towards the back.
   * The pitch (per encoders) is a difference between current encoder value and a "zero degree" encoder value
   * divided by ARM_ENCODER_CHANGE_PER_DEGREE
   * @return - degrees angle
   */
  public double getArmAngleSI() {
    return (armEncoderLeading.getPosition() - armEncoderZero) / Arm.ARM_ENCODER_CHANGE_PER_DEGREE;
  }

  // Encoder telemetry
  public double getArmEncoderLeft() {
    return armEncoderLeft.getPosition();
  }

  public double getArmEncoderRight() {
    return armEncoderRight.getPosition();
  }

  public double getArmEncoderLeading() {
    return armEncoderLeading.getPosition();
  }

  /**
   * Set Arm motor to degrees Angle using PID
   * Positive Pitch angle increases is when arm is going from front towards the back.
   * @param angle
   */
  public void setArmMotorAnglesSI(double angle) {
    armMotorLeader.getPIDController().setReference(
      // armEncoderZero is encoder position at ZERO degrees
      // So, the expected encoder position is armEncoderZero plus
      // the degrees angle multiplied by ARM_ENCODER_CHANGE_PER_DEGREE
      ( Arm.ARM_ENCODER_CHANGE_PER_DEGREE * angle) + armEncoderZero,
      ControlType.kPosition
    );
  }

  public void stopArmPID() {
    armMotorLeader.getPIDController().setReference((0), ControlType.kVoltage);
  }

  // ==================================
  // test methods; for calibration only
  // ==================================
  public void runArmMotors(double power) {
    armMotorLeader.set(power);
  }

  public void stopArmMotors() {
    armMotorLeader.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
