// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimbMotorConstantsEnum;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.GPMConstants.Arm;
import frc.robot.Constants.GPMConstants.Arm.ArmMotorConstantsEnum;
import frc.robot.Constants.GPMConstants.Intake;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonSRX frontClimberMotor; // TalonSRX
    private WPI_TalonSRX backClimberMotor; // TalonSRX
    private WPI_TalonSRX leadClimberMotor; // TalonSRX

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    // Check if need to initialize climber
    if (! EnabledSubsystems.climber) { return; }

    frontClimberMotor = new WPI_TalonSRX(ClimbMotorConstantsEnum.FRONTMOTOR.getArmMotorID());
    backClimberMotor = new WPI_TalonSRX(ClimbMotorConstantsEnum.BACKMOTOR.getArmMotorID());

    configureClimbMotors(frontClimberMotor, ClimbMotorConstantsEnum.FRONTMOTOR, null);
    configureClimbMotors(backClimberMotor, ClimbMotorConstantsEnum.BACKMOTOR, frontClimberMotor);

  }
 
  private void configureClimbMotors(WPI_TalonSRX motor, ClimbMotorConstantsEnum c, WPI_TalonSRX motorToFollow) {

        motor.configFactoryDefault();
        motor.setSafetyEnabled(false);
        // Configure motor and controller
        motor.setInverted(Intake.INTAKE_INVERTED);

    // sets which motor is the leader and follower; set follower inversion if needed
    if (c.getArmMotorFollower()) {
      motor.follow(motorToFollow);
    } else {
      leadClimberMotor = motor;
    }
  }

  public void climbUP(double power) {
    leadClimberMotor.set(power);
  }

  public void climbDown(double power) {
    leadClimberMotor.set(-power);
  }

  public void stopClimbMotor() {
    leadClimberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
