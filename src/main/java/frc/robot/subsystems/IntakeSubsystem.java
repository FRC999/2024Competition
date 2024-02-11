// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GPMConstants.Intake;

@SuppressWarnings({ "removal" })

public class IntakeSubsystem extends SubsystemBase {

  // 775 connected to TalonSRX
  private WPI_TalonSRX intakeMotor; // TalonSRX

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    // ==========================
    // === INTAKE initiatization
    // ==========================

    intakeMotor = new WPI_TalonSRX(Intake.INTAKE_MOTOR_CAN_ID);

    configureIntakeMotor();

    System.out.println("*** Intake initialized");

  }

  private void configureIntakeMotor() {

    intakeMotor.configFactoryDefault();
    intakeMotor.setSafetyEnabled(false);

    // Configure motor and controller
    intakeMotor.setInverted(Intake.INTAKE_INVERTED);

    // We may not have any encoder on the intake, so no encoder of any kind
    // TODO: Check if intake motor has encoder

    /*
     * intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.
     * CTRE_MagEncoder_Absolute);
     * intakeMotor.setSensorPhase(Intake.INTAKE_SENSOR_PHASE);
     * 
     * intakeMotor.configNeutralDeadband(Intake.INTAKE_NEUTRAL_DEADBAND,
     * Intake.INTAKE_TIMEOUT);
     * 
     * // PID Configuration
     * intakeMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10,
     * Intake.INTAKE_TIMEOUT);
     * intakeMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10,
     * Intake.INTAKE_TIMEOUT);
     * 
     * intakeMotor.configPeakOutputForward(+1.0, Intake.INTAKE_TIMEOUT);
     * intakeMotor.configPeakOutputReverse(-1.0, Intake.INTAKE_TIMEOUT);
     * intakeMotor.configNominalOutputForward(0, Intake.INTAKE_TIMEOUT);
     * intakeMotor.configNominalOutputReverse(0, Intake.INTAKE_TIMEOUT);
     * 
     * intakeMotor.selectProfileSlot(IntakePIDConstants.intakeSlot0,
     * IntakePIDConstants.pidIntakeIdx);
     * intakeMotor.config_kP(IntakePIDConstants.intakeSlot0, IntakePIDConstants.kP,
     * Intake.INTAKE_TIMEOUT);
     * intakeMotor.config_kI(IntakePIDConstants.intakeSlot0, IntakePIDConstants.kI,
     * Intake.INTAKE_TIMEOUT);
     * intakeMotor.config_kD(IntakePIDConstants.intakeSlot0, IntakePIDConstants.kD,
     * Intake.INTAKE_TIMEOUT);
     * intakeMotor.config_kF(IntakePIDConstants.intakeSlot0, IntakePIDConstants.kF,
     * Intake.INTAKE_TIMEOUT);
     * 
     * intakeMotor.config_IntegralZone(IntakePIDConstants.intakeSlot0,
     * IntakePIDConstants.Izone,
     * Intake.INTAKE_TIMEOUT);
     * intakeMotor.configClosedLoopPeakOutput(IntakePIDConstants.intakeSlot0,
     * IntakePIDConstants.PeakOutput,
     * Intake.INTAKE_TIMEOUT);
     * intakeMotor.configAllowableClosedloopError(IntakePIDConstants.intakeSlot0,
     * IntakePIDConstants.DefaultAcceptableError,
     * Intake.INTAKE_TIMEOUT);
     * 
     * intakeMotor.configClosedLoopPeriod(IntakePIDConstants.intakeSlot0,
     * IntakePIDConstants.closedLoopPeriod,
     * Intake.INTAKE_TIMEOUT);
     * 
     * intakeMotor.configMotionAcceleration(IntakePIDConstants.Acceleration,
     * Intake.INTAKE_TIMEOUT);
     * intakeMotor.configMotionCruiseVelocity(IntakePIDConstants.CruiseVelocity,
     * Intake.INTAKE_TIMEOUT);
     * intakeMotor.configMotionSCurveStrength(IntakePIDConstants.intakeSmoothing);
     * 
     */

  }

  /**
   * Run intake motor at the specified speed
   * 
   * @param speed
   */
  public void runIntake(double speed) {
    intakeMotor.set(speed);
    System.out.println("========== Intake Motor running at " + speed);
  }

  /**
   * Stop rotating the intake
   */
  public void stopIntake() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
