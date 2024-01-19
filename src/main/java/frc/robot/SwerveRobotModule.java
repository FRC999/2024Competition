package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstantsForSwerveModules.SRXAngle;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.Constants.SwerveChassis.SwerveTelemetry;
import frc.robot.Constants.SwerveChassis.TalonFXSwerveConfiguration;

@SuppressWarnings({ "deprecation", "removal" })
public class SwerveRobotModule {

    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;
    private WPI_TalonFX motorTalonFX;

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

        driveMotor.configureDriveMotor(moduleConstants);
        angleMotor.configureAngleMotor(moduleConstants);

        lastAngle = getState().angle.getDegrees();
    }

    public SwerveModuleState getState() {
        double velocity = driveMotor.getDriveEncoderVelocitySI();
        Rotation2d angle = Rotation2d.fromDegrees(angleMotor.getAngleEncoderPositionSI());
        return new SwerveModuleState(velocity, angle);
    }

    public double telemetryAngleEncoder() {
        return angleMotor.getAngleEncoderPosition();
    }

    public double telemetryAngleEncoderSI() {
        return angleMotor.getAngleEncoderPositionSI();
    }

    public double telemetryDriveEncoder() {
        return driveMotor.getDriveEncoderPosition();
    }

    public void testDriveMotorApplyPower(double power) {
        driveMotor.testMotorApplyPower(power);
    }

    public void testAngleMotorApplyPower(double power) {
        angleMotor.testMotorApplyPower(power);
    }

    public void DriveMotorApplyPower(double power) {
        driveMotor.applyPower(power);
    }

    public void AngleMotorApplyPower(double power) {
        angleMotor.applyPower(power);
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        switch (SwerveTelemetry.swerveDriveOrTelemetry) {
            case DRIVE_ONLY:

                driveMotor.applyPower(desiredState.speedMetersPerSecond / SwerveChassis.MAX_VELOCITY);
                angleMotor.setAngleMotorChassisAngleSI(desiredState.angle.getDegrees()); // Rotation2d angle does not
                                                                                         // give degrees
                break;
            case TELEMETRY_ONLY:
                printSwerveModuleState(desiredState);
                break;
            case DRIVE_AND_TELEMETRY:
                printSwerveModuleState(desiredState);

                driveMotor.applyPower(desiredState.speedMetersPerSecond / SwerveChassis.MAX_VELOCITY);
                angleMotor.setAngleMotorChassisAngleSI(desiredState.angle.getDegrees()); // Rotation2d angle does not
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
        double position = driveMotor.getDriveEncoderPositionSI();
        Rotation2d angle = currentAngle;
        return new SwerveModulePosition(position, angle);
    }

    public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstantsEnum c) {

        motorTalonFX.configFactoryDefault();
        motorTalonFX.setInverted(c.isDriveMotorInverted());

        // Encoder configuration
        motorTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,
                Constants.SwerveChassis.TalonFXSwerveConfiguration.kPIDLoopIdx,
                Constants.SwerveChassis.TalonFXSwerveConfiguration.configureTimeoutMs);

        motorTalonFX.setSensorPhase(c.getDriveMotorSensorPhase());

        configureCurrentLimiterDrive();

        motorBrakeMode();

    }

    public void configureAngleMotor(SwerveModuleConstantsEnum c) {

        motorTalonFX.configFactoryDefault();
        motorTalonFX.setInverted(c.isAngleMotorInverted());

        /**
         * Configure encoder
         * We use CTR Mag encoders directly connected to the TalonSRX.
         * These encoders can be used as both absolute and relative encoders at the same
         * time.
         */

         //TODO: Check what TalonFXFeedbackDevice.RemoteSensor0 actually means.
        motorTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,
                Constants.SwerveChassis.TalonFXSwerveConfiguration.kPIDLoopIdx,
                Constants.SwerveChassis.TalonFXSwerveConfiguration.configureTimeoutMs);

        motorTalonFX.setSensorPhase(c.getAngleMotorSensorPhase());

        initQuadrature();

        configureMotionMagicAngle(c);

        setEncoderforWheelCalibration(c);

        configureCurrentLimiterAngle();

        motorBrakeMode();

        // setAngleMotorChassisAngle(0); //Initialization turn wheels to 0 degrees

    }

    public double getDriveEncoderPosition() {
        return motorTalonFX.getSelectedSensorPosition();
    }

    public double getAngleEncoderPosition() {
        return motorTalonFX.getSelectedSensorPosition();
    }

    public double getDriveEncoderVelocity() {
        return motorTalonFX.getSelectedSensorVelocity();
    }

    public double getAngleEncoderVelocity() {
        return motorTalonFX.getSelectedSensorVelocity();
    }

    public double getDriveEncoderPositionSI() {
        return motorTalonFX.getSelectedSensorPosition()
                * Constants.SwerveChassis.TalonFXSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderPositionSI() {
        return motorTalonFX.getSelectedSensorPosition()
                * Constants.SwerveChassis.TalonFXSwerveConfiguration.degreePerTick;
    }

    public double getDriveEncoderVelocitySI() {
        return motorTalonFX.getSelectedSensorVelocity() * 10.0
                * Constants.SwerveChassis.TalonFXSwerveConfiguration.metersPerTick;
    }

    public double getAngleEncoderVelocitySI() {
        return motorTalonFX.getSelectedSensorVelocity() * 10.0
                * Constants.SwerveChassis.TalonFXSwerveConfiguration.degreePerTick;
    }

    private int getDriveAbsEncoder() {
        return (int) motorTalonFX.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    }

    public void setAngleMotorChassisAngleSI(double angle) {
        motorTalonFX.set(TalonSRXControlMode.MotionMagic, degreesToTicks(angle));
    }

    public void testMotorApplyPower(double power) {
        motorTalonFX.set(TalonSRXControlMode.PercentOutput, power);
    }

    public void applyPower(double power) {
        motorTalonFX.set(TalonSRXControlMode.PercentOutput, power);
    }

    private double degreesToTicks(double degrees) {
        return degrees / TalonFXSwerveConfiguration.degreePerTick;
    }

    private void configureMotionMagicAngle(Constants.SwerveChassis.SwerveModuleConstantsEnum c) {

        // Disable motor safety so we can use hardware PID
        motorTalonFX.setSafetyEnabled(false);

        motorTalonFX.configNeutralDeadband(SRXAngle.NeutralDeadband, 30);

        motorTalonFX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, SRXAngle.periodMs, SRXAngle.timeoutMs);
        motorTalonFX.setStatusFramePeriod(StatusFrame.Status_10_Targets, SRXAngle.periodMs, SRXAngle.timeoutMs);

        motorTalonFX.configPeakOutputForward(+1.0, SRXAngle.timeoutMs);
        motorTalonFX.configPeakOutputReverse(-1.0, SRXAngle.timeoutMs);
        motorTalonFX.configNominalOutputForward(0, SRXAngle.timeoutMs);
        motorTalonFX.configNominalOutputReverse(0, SRXAngle.timeoutMs);

        /* FPID Gains */
        motorTalonFX.selectProfileSlot(SRXAngle.SLOT_0, 0);
        motorTalonFX.config_kP(SRXAngle.SLOT_0, SRXAngle.kP, SRXAngle.timeoutMs);
        motorTalonFX.config_kI(SRXAngle.SLOT_0, SRXAngle.kI, SRXAngle.timeoutMs);
        motorTalonFX.config_kD(SRXAngle.SLOT_0, SRXAngle.kD, SRXAngle.timeoutMs);
        motorTalonFX.config_kF(SRXAngle.SLOT_0, SRXAngle.kF, SRXAngle.timeoutMs);

        motorTalonFX.config_IntegralZone(SRXAngle.SLOT_0, SRXAngle.Izone, SRXAngle.timeoutMs);
        motorTalonFX.configClosedLoopPeakOutput(SRXAngle.SLOT_0, SRXAngle.PeakOutput, SRXAngle.timeoutMs);
        motorTalonFX.configAllowableClosedloopError(SRXAngle.SLOT_0, SRXAngle.DefaultAcceptableError,
                SRXAngle.timeoutMs);

        motorTalonFX.configClosedLoopPeriod(SRXAngle.SLOT_0, SRXAngle.closedLoopPeriod, SRXAngle.timeoutMs);

        motorTalonFX.configMotionAcceleration(SRXAngle.Acceleration, SRXAngle.timeoutMs);
        motorTalonFX.configMotionCruiseVelocity(SRXAngle.CruiseVelocity, SRXAngle.timeoutMs);
        motorTalonFX.configMotionSCurveStrength(SRXAngle.Smoothing);
    }

    // Current limiter configuration for the angle motor
    private void configureCurrentLimiterAngle() {
        motorTalonFX.configPeakCurrentLimit(TalonFXSwerveConfiguration.anglePeakCurrentLimit,
                TalonFXSwerveConfiguration.configureTimeoutMs);
        motorTalonFX.configPeakCurrentDuration(TalonFXSwerveConfiguration.anglePeakCurrentDuration,
                TalonFXSwerveConfiguration.configureTimeoutMs);
        motorTalonFX.configContinuousCurrentLimit(TalonFXSwerveConfiguration.angleContinuousCurrentLimit,
                TalonFXSwerveConfiguration.configureTimeoutMs);
        motorTalonFX.enableCurrentLimit(TalonFXSwerveConfiguration.angleEnableCurrentLimit); // Honor initial setting

    }

    // Current limiter configuration for the drive motor
    private void configureCurrentLimiterDrive() {
        motorTalonFX.configPeakCurrentLimit(TalonFXSwerveConfiguration.drivePeakCurrentLimit,
                TalonFXSwerveConfiguration.configureTimeoutMs);
        motorTalonFX.configPeakCurrentDuration(TalonFXSwerveConfiguration.drivePeakCurrentDuration,
                TalonFXSwerveConfiguration.configureTimeoutMs);
        motorTalonFX.configContinuousCurrentLimit(TalonFXSwerveConfiguration.driveContinuousCurrentLimit,
                TalonFXSwerveConfiguration.configureTimeoutMs);
        motorTalonFX.enableCurrentLimit(TalonFXSwerveConfiguration.driveEnableCurrentLimit); // Honor initial setting

    }

    public void initQuadrature() { // Set absolute encoders
        int pulseWidth = motorTalonFX.getSensorCollection().getPulseWidthPosition();

        if (TalonFXSwerveConfiguration.kDiscontinuityPresent) {

            /* Calculate the center */
            int newCenter;
            newCenter = (TalonFXSwerveConfiguration.kBookEnd_0 + TalonFXSwerveConfiguration.kBookEnd_1) / 2;
            newCenter &= 0xFFF;

            /**
             * Apply the offset so the discontinuity is in the unused portion of
             * the sensor
             */
            pulseWidth -= newCenter;
        }
    }

    /**
     * The CTR Mag encoders we use to track wheel angle can be used in both absolute
     * and relative modes
     * at the same time. The Hardware PID on the TalonSRX, however, is easier to use
     * with relative encoders.
     * So, we read absolute encoders at the start, and set relative encoders so
     * their 0 corresponds to the
     * wheels being lined up and facing forward (0 degree from the forward robot
     * direction).
     * We have not found significant drift/discrepancy between absolute and relative
     * encoder increments, so
     * we do not currently recalibrate relative encoders again during the game.
     * Note that the wheels do not need to be set "forward" at the beginning of the
     * game. The absolute encoder
     * will set the right angle-related value to the relative encoder, since
     * absolute encoders are not set to 0 after
     * power cycle. The drive routines will then change the wheel positions as
     * needed.
     */
    public void setEncoderforWheelCalibration(SwerveModuleConstantsEnum c) {
        double difference = getDriveAbsEncoder() - c.getAngleOffset() * 4096.0 / 360.0;
        double encoderSetting = 0.0;

        if (difference < 0) {
            difference += TalonFXSwerveConfiguration.clicksSRXPerFullRotation;
        }

        if (difference <= TalonFXSwerveConfiguration.clicksSRXPerFullRotation / 2) {
            encoderSetting = difference;

        } else {
            encoderSetting = difference - TalonFXSwerveConfiguration.clicksSRXPerFullRotation;
        }

        motorTalonFX.setSelectedSensorPosition(encoderSetting);

        System.out.println("Set encoder for motor " + c.getAngleMotorID() + " to " + encoderSetting);

    }

    private void motorBrakeMode() {
        motorTalonFX.setNeutralMode(NeutralMode.Brake);
    }

}
