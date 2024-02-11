// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GPMConstants.Shooter.ShooterMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Shooter.ShooterPIDConstants;

public class ShooterSubsystem extends SubsystemBase {

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

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    shooterMotorLeft = new CANSparkMax(ShooterMotorConstantsEnum.LEFTMOTOR.getShooterMotorID(), MotorType.kBrushless);
    shooterMotorRight = new CANSparkMax(ShooterMotorConstantsEnum.RIGHTMOTOR.getShooterMotorID(), MotorType.kBrushless);

    shooterEncoderLeft = shooterMotorLeft.getEncoder();
    shooterEncoderRight = shooterMotorRight.getEncoder();

    // Main Motor; should not follow the other motor
    configureshooterMotors(shooterMotorLeft, shooterPIDControllerLeft, ShooterMotorConstantsEnum.LEFTMOTOR, null);
    // Follower Motor
    configureshooterMotors(shooterMotorRight, shooterPIDControllerRight, ShooterMotorConstantsEnum.RIGHTMOTOR,
        shooterMotorLeft);

    System.out.println("*** Shooter initialized");

  }

  /**
   * Configure Shooter motors with a main and a follower
   * 
   * @param motor         - motor object
   * @param p             - PID controller object
   * @param c             - motor constants
   * @param motorToFollow - motor to follow if this is a follower
   */
  private void configureshooterMotors(CANSparkMax motor, SparkPIDController p, ShooterMotorConstantsEnum c,
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

  public void runShooter(double speed) {
    shooterMotorLeader.getPIDController().setReference((speed), ControlType.kVelocity);
    System.out.println("========== Shooter Motor running at " + speed);
  }

  public void stopShooter() {
    shooterMotorLeader.getPIDController().setReference((0), ControlType.kVelocity);
    System.out.println("========== Shooter Motor stopped ");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
