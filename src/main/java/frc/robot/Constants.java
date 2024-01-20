// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class GPMConstants {
    public static final int INTAKE_MOTOR_CAN_ID = 0;
    public static final int LEFT_ARM_CAN_ID = 0;
    public static final int RIGHT_ARM_CAN_ID = 0;    

    public static final int SHOOTER_A_CAN_ID = 0;    
    public static final int SHOOTER_B_CAN_ID = 0;

    public static final int NOTE_DETECTION_CAN_ID = 0;
  }
  public static class IMUConstants {
    public static final int PIGEON2_CAN_ID = 0;    
  }

  public static final class SwerveChassis {

    
		// TRAJECTORY PARAMETERS 3039
		/* Drive Feedforward */
		public static final double DRIVE_KS = 0.11937 / 12;
		public static final double DRIVE_KV = 2.6335 / 12;
		public static final double DRIVE_KA = 0.46034 / 12;

		/*
		 * Drive train properties
		 * All measurements are in meters
		 */
		public static final double TRACK_WIDTH = 0.385; // left to right
		public static final double WHEEL_BASE = 0.44; // front to back
		public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

		/**
		 * This class lists locating of each of the swerve modules from the center of
		 * the robot.
		 * It is assumed that there are four swerve modules located at the edges of a
		 * rectangle.
		 * It is also assumed that the center of rotation is at the center of that
		 * rectangle.
		 * 
		 * If your center of rotation is far off the center of that rectangle, which may
		 * happen
		 * due to the very scewed CG or uneven traction, you may want to adjust the
		 * numbers below
		 * based on the center of rotation.
		 * The order of the wheels location definition must match the order of the
		 * swerve modules
		 * defined in the DriveSubsystem for the SwerveModule array.
		 */
		public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

		/*
		 * Ramp Rates and Current Limits. Assumed to be the same for all drivetrain
		 * motors of the
		 * same type/purpose.
		 */
		public static final double DRIVE_CLOSED_LOOP_RAMP = 0;
		public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
		public static final int ANGLE_MOTOR_SMART_CURRENT = 25;
		public static final double ANGLE_MOTOR_SECONDARY_LIMIT = 40;
		public static final int DRIVE_MOTOR_SMART_CURRENT = 40;
		public static final double DRIVE_MOTOR_SECONDARY_LIMIT = 60;

		/**
		 * Angle Motor PID. Assumed to be the same for all angle motors
		 * These PID constants are only used for auto trajectory driving, and not
		 * teleop.
		 * Changes to these constants will have a substantial impact on the precision of
		 * your
		 * trajectory if it includes holonomic rotation.
		 * Make sure to test the values and adjust them as needed for your robot.
		 */
		public static final double ANGLE_CHASSIS_KP = 6.25;
		public static final double ANGLE_CHASSIS_KI = 0.4;
		public static final double ANGLE_CHASSIS_KD = 0.7;

		public static final double ANGLE_MOTOR_MIN_OUTPUT = -1;
		public static final double ANGLE_MOTOR_MAX_OUTPUT = 1;

		public static final double ANGLE_MOTOR_PID_TIMEOUT = 30; // milliseconds

		public static final double ANGLE_MOTOR_VELOCITY_CONVERSION = (360.0 * 10.0) / 4096.0; // conversion factor from
																								// tick/100ms to
																								// degree/s

		/**
		 * Drive Motor PID. Assumed to be the same for all drive motors
		 * These PID constants are only used for auto trajectory driving, and not
		 * teleop.
		 * We found that changing them a bit will not have a substantial impact on the
		 * trajectory with PathPlanner
		 * even if a trajectory includes a holonomic component.
		 */
		public static final double DRIVE_CHASSIS_KP = 3.0;
		public static final double DRIVE_CHASSIS_KI = 0.05;
		public static final double DRIVE_CHASSIS_KD = 0.01;

		/**
		 * Maximum linear speed of chassis in meters per second
		 * Note that not determining this number precisely up front will not affect your
		 * teleop driving
		 * as the teleop logic will simply use it as a point of reference.
		 * Changing this number will not require any other changes in the teleop code.
		 */
		public static final double MAX_VELOCITY = 3.0;

		/**
		 * Radians per second.
		 * Swerve chassis assumes that the maximum linear speed during rotation is the
		 * same as the
		 * maxiumum linear speed during forward drive.
		 * That means the maximum angular speed can be calculated by dividing the
		 * maximum linear speed by
		 * the radius of rotation, which can be calculated by halving the distance
		 * between the opposing swerve
		 * modules such as the front left and rear right.
		 */
		public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY
				/ (Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2);

		// For trajectory driving.
		public static final double MAX_ACCELERATION = 2.0;

		/**
		 * Parameters for BaseMotorTalonFX class
		 * This class is specific to the motors controlled by TalonFX controller.
		 * Parameters specified here are primarily used in the motor configuration
		 * methods, as well as
		 * getters that translate encoder outputs from encoder-specific units to the SI
		 * units.
		 * Other motor controller implementations will likely have a different set of
		 * constants.
		 */
		public static final class TalonFXSwerveConfiguration {

			// We assume that all TalonFX controlers need the same PID and some other
			// hardware configuration parameters
			public static final int kPIDLoopIdx = 0; // Talon Loop ID
			public static final int configureTimeoutMs = 30; // Hardware Talon Configuration TimeoutMs (probably need to
																// be larger than CAN cycle of 20ms)
																// For Talon configuration command it means - you expect
																// a reply from the controller
																// in that time; otherwise assume the error

			// Customize the following values to your prototype
			public static final double metersPerTickFX = 1.0 / 1462.25; // TODO: measure this number on the robot - drive motor only
			public static final double degreePerTickFX = 360.0 / 4096.0; 	// Angle motor only
																		// On our swerve prototype 1 angular rotation of
																		// the wheel = 1 full rotation of the encoder
			public static final double degreePerTickCancoder = 360.0 / 4096.0 ; // degrees per tick for cancoder to track absolute wheel position

			// Absolute encoder setup
			public static final boolean kDiscontinuityPresent = true;
			public static final int kBookEnd_0 = 910; /* 80 deg */
			public static final int kBookEnd_1 = 1137; /* 100 deg */
			public static final int clicksFXPerFullRotation = 4096; // rollover on 999 swerve encoder - we use Falcon FX
																		// relative encoders for angle with x:x ratio
																		//TODO: find out and fix that ratio

			/**
			 * Current limiters
			 * 
			 * In TalonFX the limiters limit the input current (not the stator current)
			 * and work in the following way:
			 * If the current demand exceeds peak current for more than a specified
			 * duration,
			 * the current will be limited to the continuous limit until the power demand
			 * drops
			 * below the continuous limit. After that the limiter "resets" and will watch
			 * for peak current again.
			 * For instance in this case if the peak demand on the angle motor exceeds 40amp
			 * for more than 1 second,
			 * the max draw on it will be limited to 25amp until the demand falls below
			 * 25amp.
			 * 
			 * The reason for the limits - you want to make sure you do not trip the
			 * breakers, and Rio will not
			 * go into brownout protection mode. Note that PDP breakers do not immediately
			 * trip when you exceed the
			 * power limit written on them.
			 * Good discussions about brownouts as well as breakers are here:
			 * https://docs.wpilib.org/en/latest/docs/software/roborio-info/roborio-brownouts.html
			 * https://www.chiefdelphi.com/t/power-draw-from-motors/368244/7
			 * https://v5.docs.ctr-electronics.com/en/latest/ch13_MC.html#new-api-in-2020
			 */

			public static final int angleContinuousCurrentLimit = 40; // amperes
			public static final int anglePeakCurrentLimit = 60; // amperes
			public static final int anglePeakCurrentDuration = 1; // Seconds
			public static final boolean angleEnableCurrentLimit = true;

			public static final double driveContinuousCurrentLimit = 40; // amperes
			public static final double drivePeakCurrentLimit = 60; // amperes
			public static final double drivePeakCurrentDuration = 0.5; // Milliseconds
			public static final boolean driveEnableCurrentLimit = true;
			public static final double driveSecondsFromNeutralToFull = 0.1; //TODO: - check if needed - ramp-up limit to avoid peaks

		}

		/**
		 * The following constants are used to print swerve telemetry. Please, note that
		 * excessive telemetry
		 * will cause excessive CPU usage, and ultimately will result in a packet loss.
		 * If after enabling telemetry you will see CPU approaching 100% in the RIO log,
		 * such settings will not be
		 * usable for extensive driving or competition. These settings were put in place
		 * so a team can
		 * troubleshoot teleop and trajectory driving.
		 */
		public static final class SwerveTelemetry {
			public static enum SwerveDriveOrTelemetry {
				DRIVE_ONLY,
				TELEMETRY_ONLY,
				DRIVE_AND_TELEMETRY;
			}

			/**
			 * Specify whether telemetry will be printed and/or the robot will apply power
			 * to the motors
			 */
			public static final SwerveDriveOrTelemetry swerveDriveOrTelemetry = SwerveDriveOrTelemetry.DRIVE_ONLY;

			/**
			 * Print odometry telemetry every 20 milliseconds.
			 */
			public static final boolean odometryTelemetryPrint = false;

		}

		/*
		 * Add controller types for each supported motor controller including simulated
		 * ones
		 * Make sure to modify BaseMotorPassthrough.java and add specific implementation
		 * class
		 * under the "Motor" folder
		 */
		public static enum BaseMotorControllerTypes {
			TALON_FX,
		}

		/**
		 * Swerve Module Constants for each module including
		 * driveMotorType - type of the motor controller (e.g. TalonFX vs NEO vs
		 * simulations)
		 * angleMotorType - type of the motor controller (e.g. TalonFX vs NEO vs
		 * simulations)
		 * driveMotorID - CAN ID of the drive motors
		 * angleMotorID - CAN ID of the rotation motors
		 * angleOffset - Angle deviation of the absolute encoder when the
		 * respective wheel is pointing forward based on the absolute encoder value
		 * driveMotorInverted - is the drive motor inverted
		 * angleMotorInverted - is the angle motor inverted
		 * driveMotorSensorPhaseInverted - is the drive motor sensor phase inverted
		 * angleMotorSensorPhaseInverted - is the angle motor sensor phase inverted
		 * 
		 * For sensor phase we should use PID rule - when the positive power is applied,
		 * the motor should propell the robot "forward" and the corresponding encoder
		 * value should increase. Also for the angle motors the "positive" direction
		 * is counterclockwise.
		 * 
		 * Only include constants that may differ for each motor.
		 * Items that are the same for each motoro or motor type (e.g. PID constants)
		 * should be defined elsewhere.
		 * 
		 * Since this is an ENUM, need to have getter method for each value.
		 */
		public static enum SwerveModuleConstantsEnum {
			MOD0( // Front Left
					BaseMotorControllerTypes.TALON_FX, // Drive motor type
					BaseMotorControllerTypes.TALON_FX, // Angle motor type
					7, // driveMotorID
					8, // angleMotorID
					(3268.0 * 360.0) / 4096.0, // angleOffset of cancoder to mark zero-position
					true, // Inversion for drive motor
					false, // Inversion for angle motor
					false, // Sensor phase for drive motor
					false, // Sensor phase for angle motor
					30 // cancoder ID
			),
			MOD1( // Front Right
					BaseMotorControllerTypes.TALON_FX, // Drive motor type
					BaseMotorControllerTypes.TALON_FX, // Angle motor type
					5, // driveMotorID
					6, // angleMotorID
					(433.0 * 360.0) / 4096.0, // angleOffset of cancoder to mark zero-position
					true, // Inversion for drive motor
					false, // Inversion for angle motor
					false, // Sensor phase for drive motor
					false, // Sensor phase for angle motor
					31 // cancoder ID

			),
			MOD2( // Back Left
					BaseMotorControllerTypes.TALON_FX, // Drive motor type
					BaseMotorControllerTypes.TALON_FX, // Angle motor type
					1, // driveMotorID
					2, // angleMotorID
					(3489.0 * 360.0) / 4096.0, // angleOffset of cancoder to mark zero-position
					true, // Inversion for drive motor
					false, // Inversion for angle motor
					true, // Sensor phase for drive motor
					false, // Sensor phase for angle motor
					32 // cancoder ID

			),
			MOD3( // Back Right
					BaseMotorControllerTypes.TALON_FX, // Drive motor type
					BaseMotorControllerTypes.TALON_FX, // Angle motor type
					3, // driveMotorID
					4, // angleMotorID
					(3307.0 * 360.0) / 4096.0, // angleOffset of cancoder to mark zero-position
					true, // Inversion for drive motor
					false, // Inversion for angle motor
					true, // Sensor phase for drive motor
					false, // Sensor phase for angle motor
					33 // cancoder ID

			);

			private BaseMotorControllerTypes driveBaseMotorControllerType;
			private BaseMotorControllerTypes angleBaseMotorControllerType;
			private int driveMotorID;
			private int angleMotorID;
			private double angleOffset;
			private boolean driveMotorInverted;
			private boolean angleMotorInverted;
			private boolean driveMotorSensorPhase;
			private boolean angleMotorSensorPhase;
			private int cancoderID;

			SwerveModuleConstantsEnum(BaseMotorControllerTypes dm, BaseMotorControllerTypes am, int d, int a, double o,
					boolean di, boolean ai, boolean ds, boolean as, int c) {
				this.driveBaseMotorControllerType = dm;
				this.angleBaseMotorControllerType = am;
				this.driveMotorID = d;
				this.angleMotorID = a;
				this.angleOffset = o;
				this.driveMotorInverted = di;
				this.angleMotorInverted = ai;
				this.driveMotorSensorPhase = ds;
				this.angleMotorSensorPhase = as;
				this.cancoderID = c;
			}

			public BaseMotorControllerTypes getDriveMotorControllerType() {
				return driveBaseMotorControllerType;
			}

			public BaseMotorControllerTypes getAngleMotorControllerType() {
				return angleBaseMotorControllerType;
			}

			public int getDriveMotorID() {
				return driveMotorID;
			}

			public int getAngleMotorID() {
				return angleMotorID;
			}

			public double getAngleOffset() {
				return angleOffset;
			}

			public boolean isDriveMotorInverted() {
				return driveMotorInverted;
			}

			public boolean isAngleMotorInverted() {
				return angleMotorInverted;
			}

			public boolean getDriveMotorSensorPhase() {
				return driveMotorSensorPhase;
			}

			public boolean getAngleMotorSensorPhase() {
				return angleMotorSensorPhase;
			}

			public int getCancoderID() {
				return cancoderID;
			}

		} // End ENUM SwerveModuleConstants

	} // End Swerve

	

	/**
	 * These Hardware PID constants are used by the individual swerve modules, and
	 * are used only by turn motors.
	 * We do not currently use Hardware PID for manual or trajectory driving.
	 */
	public static final class PIDConstantsForSwerveModules {

		// Hardware PID-related constants for angle motors controlled by TalonFX
		public static final class FXAngle {

			public static final int SLOT_0 = 0;
			public static final double kP = 0.75;
			public static final double kI = 0.005;
			public static final double kD = 0.01;
			public static final double kF = 0;
			public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
			public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
			public static final int Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for
													// S-Curve (greater value yields greater smoothing).
			public static final double DefaultAcceptableError = 5; // Sensor units
			public static final double Izone = 500;
			public static final double PeakOutput = 0.5; // Closed Loop peak output
			public static final double NeutralDeadband = 0.001;
			public static final int periodMs = 10; // status frame period
			public static final int timeoutMs = 30; // status frame timeout
			public static final int closedLoopPeriod = 1; // 1ms for TalonFX and locally connected encoder

		}

	}

	/**
	 * Controller-related constants.
	 * Here we define port numbers, axis, deadbands, button numbers and various
	 * ability flags, such as use of the cube driving
	 */
	public class OIConstants {
    public static final int driverControllerPort = 0;
    public static final int robotCentricButton = 2;

    public static final int XBOX_CONTROLLER_PORT = 4;
    public static final double XBOX_DEADBAND_X = 0.03;
    public static final double XBOX_DEADBAND_Y = 0.03;
    public static final double XBOX_DEADBAND_OMEGA = 0.03;
    public static final boolean XBOX_CUBE_CONTROLLER_LEFT_STICK = false;
    public static final boolean XBOX_CUBE_CONTROLLER_RIGHT_STICK = false;
}


}
