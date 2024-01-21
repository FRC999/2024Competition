package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstantsForSwerveModules.FXAngle;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.Constants.SwerveChassis.SwerveTelemetry;
import frc.robot.Constants.SwerveChassis.TalonFXSwerveConfiguration;

@SuppressWarnings({ "deprecation", "removal" })
public class SwerveRobotModule {

    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;

    private WPI_CANCoder cancoder;

    private int moduleNumber;
    private double angleOffset;
    private double lastAngle;

    private Rotation2d currentAngle = new Rotation2d();

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveChassis.DRIVE_KS,
            Constants.SwerveChassis.DRIVE_KV, Constants.SwerveChassis.DRIVE_KA);

    public SwerveRobotModule(int moduleNumber, SwerveModuleConstantsEnum moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.getAngleOffset();

        driveMotor = new WPI_TalonFX(moduleConstants.getDriveMotorID());
        angleMotor = new WPI_TalonFX(moduleConstants.getAngleMotorID());

        cancoder = new WPI_CANCoder(moduleConstants.getCancoderID());

        configureDriveMotor(moduleConstants);
        configureAngleMotor(moduleConstants);

        lastAngle = getState().angle.getDegrees();
    }

    public SwerveModuleState getState() {
        double velocity = getDriveEncoderVelocitySI();
        Rotation2d angle = Rotation2d.fromDegrees(getAngleEncoderPositionSI());
        return new SwerveModuleState(velocity, angle);
    }

    public double telemetryAngleEncoder() {
        return getAngleEncoderPosition();
    }

    public double telemetryAngleEncoderSI() {
        return getAngleEncoderPositionSI();
    }

    public double telemetryAngleEncoderSIAbs() {
        return getAngleEncoderPositionSIAbs();
    }

    public double telemetryDriveEncoder() {
        return getDriveEncoderPosition();
    }

    public double telemetryCANCoderSI() {
        return cancoder.getAbsolutePosition();
    }

    public void testDriveMotorApplyPower(double power) {
        driveMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void testAngleMotorApplyPower(double power) {
        angleMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void driveMotorApplyPower(double power) {
        driveMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void angleMotorApplyPower(double power) {
        angleMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        switch (SwerveTelemetry.swerveDriveOrTelemetry) {
            case DRIVE_ONLY:

                driveMotorApplyPower(desiredState.speedMetersPerSecond / SwerveChassis.MAX_VELOCITY);
                setAngleMotorChassisAngleSI(desiredState.angle.getDegrees()); // Rotation2d angle does not
                                                                                         // give degrees
                break;
            case TELEMETRY_ONLY:
                printSwerveModuleState(desiredState);
                break;
            case DRIVE_AND_TELEMETRY:
                printSwerveModuleState(desiredState);

                driveMotorApplyPower(desiredState.speedMetersPerSecond / SwerveChassis.MAX_VELOCITY);
                setAngleMotorChassisAngleSI(desiredState.angle.getDegrees()); // Rotation2d angle does not
                                                                                         // give degrees
                break;
        }
    }

    public void setDesiredStateCalibration(SwerveModuleState desiredState) {

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        printSwerveModuleState(desiredState);
    }

    public void printSwerveModuleState(SwerveModuleState moduleState) {
        System.out.print(" SM: " + moduleNumber);
        System.out.print(" P: " + moduleState.speedMetersPerSecond / SwerveChassis.MAX_VELOCITY);
        System.out.println(" A: " + moduleState.angle.getDegrees());
    }

    public SwerveModulePosition getPosition() {
        double position = getDriveEncoderPositionSI();
        Rotation2d angle = currentAngle;
        return new SwerveModulePosition(position, angle);
    }

    public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstantsEnum c) {

        driveMotor.configFactoryDefault();
        driveMotor.setInverted(c.isDriveMotorInverted());

        // Encoder configuration
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.SwerveChassis.TalonFXSwerveConfiguration.kPIDLoopIdx,
                Constants.SwerveChassis.TalonFXSwerveConfiguration.configureTimeoutMs);

        driveMotor.setSensorPhase(c.getDriveMotorSensorPhase());

        configureCurrentLimiterDrive();

        driveMotorBrakeMode();

    }

    public void configureAngleMotor(SwerveModuleConstantsEnum c) {

        angleMotor.configFactoryDefault();
        angleMotor.setInverted(c.isAngleMotorInverted());

        /**
         * Configure encoder
         * We use CTR Mag encoders directly connected to the TalonSRX.
         * These encoders can be used as both absolute and relative encoders at the same
         * time.
         */

         //TODO: Check what TalonFXFeedbackDevice.RemoteSensor0 actually means.
        angleMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.SwerveChassis.TalonFXSwerveConfiguration.kPIDLoopIdx,
                Constants.SwerveChassis.TalonFXSwerveConfiguration.configureTimeoutMs);

        angleMotor.setSensorPhase(c.getAngleMotorSensorPhase());

        configureMotionMagicAngle(c);

        setEncoderforWheelCalibration(c);

        //configureCurrentLimiterAngle();

        angleMotorBrakeMode();

        // Turn all wheels straight when the robot is turned ON
        //if (c.getAngleMotorID() == 2) {
        setAngleMotorChassisAngleSI(0); //Initialization turn wheels to 0 degrees
        //}

    }

    public double getDriveEncoderPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    /**
     * Get encoder value in RAW units
     * @return
     */
    public double getAngleEncoderPosition() {
        return angleMotor.getSelectedSensorPosition();
    }

    public double getDriveEncoderVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getAngleEncoderVelocity() {
        return angleMotor.getSelectedSensorVelocity();
    }

    public double getDriveEncoderPositionSI() {
        return driveMotor.getSelectedSensorPosition()
                * TalonFXSwerveConfiguration.metersPerTickFX;
    }

    public double getAngleEncoderPositionSI() {
        return angleMotor.getSelectedSensorPosition()
                * TalonFXSwerveConfiguration.degreePerTickFX;
    }

    public double getAngleEncoderPositionSIAbs() {
        return (angleMotor.getSelectedSensorPosition()
                * TalonFXSwerveConfiguration.degreePerTickFX)
                %360;
    }

    /**
     * Retuns a value in m/s
     * @return
     */
    public double getDriveEncoderVelocitySI() {
        return driveMotor.getSelectedSensorVelocity() * 10.0 // convert from RAW units, which are per 100ms
                * Constants.SwerveChassis.TalonFXSwerveConfiguration.metersPerTickFX;
    }

    /**
     * Retuns a value in m/s
     * @return
     */
    public double getAngleEncoderVelocitySI() {
        return angleMotor.getSelectedSensorVelocity() * 10.0 // convert from RAW units, which are per 100ms 
                * Constants.SwerveChassis.TalonFXSwerveConfiguration.degreePerTickFX;
    }

    private double getCancoderAbsEncoderValue() {
        return cancoder.getAbsolutePosition();
    }

    public void setAngleMotorChassisAngleSI(double angle) {
        //angleMotor.set(TalonFXControlMode.MotionMagic, degreesToTicks(angle));
        angleMotor.set(TalonFXControlMode.Position, degreesToTicks(angle));
    }

    public void applyPower(double power) {
        angleMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    private double degreesToTicks(double degrees) {
        return degrees / TalonFXSwerveConfiguration.degreePerTickFX;
    }

    /**
     * Configure MotionMagic for Angle motor
     * @param c
     */
    private void configureMotionMagicAngle(Constants.SwerveChassis.SwerveModuleConstantsEnum c) {

        // Disable motor safety so we can use hardware PID
        angleMotor.setSafetyEnabled(false);

        angleMotor.configNeutralDeadband(FXAngle.NeutralDeadband, 30);

        //TODO: Recheck all of these parameters and set them for FALCON motors

        angleMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, FXAngle.periodMs, FXAngle.timeoutMs);
        angleMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, FXAngle.periodMs, FXAngle.timeoutMs);

        angleMotor.configPeakOutputForward(+1.0, FXAngle.timeoutMs);
        angleMotor.configPeakOutputReverse(-1.0, FXAngle.timeoutMs);
        angleMotor.configNominalOutputForward(0, FXAngle.timeoutMs);
        angleMotor.configNominalOutputReverse(0, FXAngle.timeoutMs);

        /* FPID Gains */
        angleMotor.selectProfileSlot(FXAngle.SLOT_0, 0);
        angleMotor.config_kP(FXAngle.SLOT_0, FXAngle.kP, FXAngle.timeoutMs);
        angleMotor.config_kI(FXAngle.SLOT_0, FXAngle.kI, FXAngle.timeoutMs);
        angleMotor.config_kD(FXAngle.SLOT_0, FXAngle.kD, FXAngle.timeoutMs);
        angleMotor.config_kF(FXAngle.SLOT_0, FXAngle.kF, FXAngle.timeoutMs);

        angleMotor.config_IntegralZone(FXAngle.SLOT_0, FXAngle.Izone, FXAngle.timeoutMs);
        angleMotor.configClosedLoopPeakOutput(FXAngle.SLOT_0, FXAngle.PeakOutput, FXAngle.timeoutMs);
        angleMotor.configAllowableClosedloopError(FXAngle.SLOT_0, FXAngle.DefaultAcceptableError,
                FXAngle.timeoutMs);

        angleMotor.configClosedLoopPeriod(FXAngle.SLOT_0, FXAngle.closedLoopPeriod, FXAngle.timeoutMs);

        angleMotor.configMotionAcceleration(FXAngle.Acceleration, FXAngle.timeoutMs);
        angleMotor.configMotionCruiseVelocity(FXAngle.CruiseVelocity, FXAngle.timeoutMs);
        angleMotor.configMotionSCurveStrength(FXAngle.Smoothing);
    }

    // Current limiter configuration for the angle motor
    private void configureCurrentLimiterAngle() {
        angleMotor.configSupplyCurrentLimit( 
            new SupplyCurrentLimitConfiguration( 
                TalonFXSwerveConfiguration.angleEnableCurrentLimit, // enable current limit for drive motor
                TalonFXSwerveConfiguration.angleContinuousCurrentLimit, // continuous draw to drop to
                TalonFXSwerveConfiguration.anglePeakCurrentLimit, // threshold after which drop to continous limit
                TalonFXSwerveConfiguration.anglePeakCurrentDuration
            )
        );
        // Not limiting OpenLoopRamp as we want the turn to run as fast as possible
    }

    // Current limiter configuration for the drive motor
    private void configureCurrentLimiterDrive() {
        driveMotor.configSupplyCurrentLimit( 
            new SupplyCurrentLimitConfiguration( 
                TalonFXSwerveConfiguration.driveEnableCurrentLimit, // enable current limit for drive motor
                TalonFXSwerveConfiguration.driveContinuousCurrentLimit, // continuous draw to drop to
                TalonFXSwerveConfiguration.drivePeakCurrentLimit, // threshold after which drop to continous limit
                TalonFXSwerveConfiguration.drivePeakCurrentDuration
            )
        );
        driveMotor.configOpenloopRamp(TalonFXSwerveConfiguration.driveSecondsFromNeutralToFull, TalonFXSwerveConfiguration.configureTimeoutMs);
    }

    /**
     * The CTR Mag encoders we use to track wheel angle can be used in both absolute
     * and relative modes
     * at the same time. The Hardware PID on the TalonFX, however, is easier to use
     * with relative encoders.
     * So, we read absolute encoders at the start, and set relative encoders so
     * their 0 corresponds to the
     * wheels being lined up and facing forward (0 degree from the forward robot
     * direction).
     * We have not found significant drift/discrepancy between absolute and relative
     * encoder increments, so
     * we do not currently recalibrate relative encoders again during the game.
     * TalonFX rotates more than ones per rotation of the wheel because of the gearbox.
     * Note that the wheels do not need to be set "forward" at the beginning of the
     * game. The absolute encoder in CANCODER
     * will set the right angle-related value to the relative encoder in TalonFX, since
     * absolute encoders are not set to 0 after
     * power cycle. The drive routines will then change the wheel positions as
     * needed.
     */
    public void setEncoderforWheelCalibration(SwerveModuleConstantsEnum c) {
        double difference = (getCancoderAbsEncoderValue() - c.getAngleOffset())
                / TalonFXSwerveConfiguration.degreePerTickFX; // cancoder returns Abs value in Degrees
        double encoderSetting = 0.0;

        // alex test
        //System.out.println(c.getAngleMotorID()+"#"+getCancoderAbsEncoderValue()+"#"+c.getAngleOffset()+"#");

        if (difference < 0) {
            difference += TalonFXSwerveConfiguration.clicksFXPerFullRotation;
        }

        if (difference <= TalonFXSwerveConfiguration.clicksFXPerFullRotation / 2) {
            encoderSetting = difference;

        } else {
            encoderSetting = difference - TalonFXSwerveConfiguration.clicksFXPerFullRotation;
        }

        angleMotor.setSelectedSensorPosition(encoderSetting);

        System.out.println("Set encoder for motor " + c.getAngleMotorID() + " to " + encoderSetting);

    }

    private void driveMotorBrakeMode() {
        driveMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void angleMotorBrakeMode() {
        angleMotor.setNeutralMode(NeutralMode.Brake);
    }


}
