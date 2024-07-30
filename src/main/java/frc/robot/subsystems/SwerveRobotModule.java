package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstantsForSwerveModules.FXAngle;
import frc.robot.Constants;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.Constants.SwerveChassis.SwerveTelemetry;
import frc.robot.Constants.SwerveChassis.TalonFXSwerveConfiguration;

@SuppressWarnings({ "removal" })
public class SwerveRobotModule extends SubsystemBase {

    private TalonFX driveMotor;
    private TalonFX angleMotor;

    private CANcoder cancoder;

    private int moduleNumber;
    private double angleOffset;
    private double lastAngle;

    private Rotation2d currentAngle = new Rotation2d();

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveChassis.DRIVE_KS,
            Constants.SwerveChassis.DRIVE_KV, Constants.SwerveChassis.DRIVE_KA);

    public SwerveRobotModule(int moduleNumber, SwerveModuleConstantsEnum moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.getAngleOffset();

        driveMotor = new TalonFX(moduleConstants.getDriveMotorID());
        angleMotor = new TalonFX(moduleConstants.getAngleMotorID());

        cancoder = new CANcoder(moduleConstants.getCancoderID());

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
        return cancoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    public double telemetryCANCoder() {
        return cancoder.getAbsolutePosition().getValueAsDouble();
    }

    public void testDriveMotorApplyPower(double power) {
        driveMotor.set(power);
    }

    public void testAngleMotorApplyPower(double power) {
        angleMotor.set(power);
    }

    public void driveMotorApplyPower(double power) {
        driveMotor.set(power);
    }

    public void angleMotorApplyPower(double power) {
        angleMotor.set(power);
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

        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;

        //driveMotor.setInverted(c.isDriveMotorInverted());

        talonFXConfigs.MotorOutput.Inverted = c.isDriveMotorInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        System.out.println("MI: "+c.getDriveMotorID()+" "+c.isDriveMotorInverted());

        // Encoder configuration
        // driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        // Constants.SwerveChassis.TalonFXSwerveConfiguration.kPIDLoopIdx,
        // Constants.SwerveChassis.TalonFXSwerveConfiguration.configureTimeoutMs);

        // driveMotor.setSensorPhase(c.getDriveMotorSensorPhase());

        configureCurrentLimiterDrive(talonFXConfigs);

        driveMotor.getConfigurator().apply(talonFXConfigs);

        driveMotorBrakeMode();

    }

    public void configureAngleMotor(SwerveModuleConstantsEnum c) {

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.Feedback.FeedbackRotorOffset = 0;
        angleMotor.getConfigurator().apply(talonFXConfigs);
        var rs = angleMotor.getRotorPosition();
        rs.waitForUpdate(0.2);
        System.out.println("E:"+getAngleEncoderPosition());
        //  return;
         
        var slot0Configs = talonFXConfigs.Slot0;

        talonFXConfigs.MotorOutput.Inverted = c.isAngleMotorInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
       // angleMotor.setInverted(c.isAngleMotorInverted());

        System.out.println("MAI: "+c.getDriveMotorID()+" "+c.isDriveMotorInverted());

        /**
         * Configure encoder
         * We use CTR Mag encoders directly connected to the TalonSRX.
         * These encoders can be used as both absolute and relative encoders at the same
         * time.
         */

        // TODO: Check what TalonFXFeedbackDevice.RemoteSensor0 actually means.
        // angleMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        // Constants.SwerveChassis.TalonFXSwerveConfiguration.kPIDLoopIdx,
        // Constants.SwerveChassis.TalonFXSwerveConfiguration.configureTimeoutMs);

        //angleMotor.setSensorPhase(c.getAngleMotorSensorPhase());

        configureMotionMagicAngle(c , talonFXConfigs);

        setEncoderforWheelCalibration(c, talonFXConfigs);

        //configureCurrentLimiterAngle();

        angleMotor.getConfigurator().apply(talonFXConfigs);

        angleMotorBrakeMode();

        // Turn all wheels straight when the robot is turned ON
        // if (c.getAngleMotorID() == 2) {
        //setAngleMotorChassisAngleSI(0); // Initialization turn wheels to 0 degrees
        // }

    }

    public double getDriveEncoderPosition() {
        return driveMotor.getRotorPosition().getValue();
    }

    /**
     * Get encoder value in RAW units
     * 
     * @return
     */
    public double getAngleEncoderPosition() {
        return angleMotor.getRotorPosition().getValueAsDouble();
    }

    public double getDriveEncoderVelocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble();
    }

    public double getAngleEncoderVelocity() {
        return angleMotor.getRotorVelocity().getValueAsDouble();
    }

    public double getDriveEncoderPositionSI() {
        return driveMotor.getRotorPosition().getValueAsDouble()
                * TalonFXSwerveConfiguration.metersPerRotationFX;
    }

    public double getAngleEncoderPositionSI() {
        return angleMotor.getRotorPosition().getValueAsDouble()
                * TalonFXSwerveConfiguration.degreePerRotationFX;
    }

    public double getAngleEncoderPositionSIAbs() {
        return (angleMotor.getRotorPosition().getValueAsDouble()
                * TalonFXSwerveConfiguration.degreePerRotationFX)
                % 360.0;
    }

    /**
     * Retuns a value in m/s
     * 
     * @return
     */
    public double getDriveEncoderVelocitySI() {
        return driveMotor.getRotorVelocity().getValueAsDouble() // convert from RAW units - rotations, which are per 1s
                * Constants.SwerveChassis.TalonFXSwerveConfiguration.metersPerRotationFX;
    }

    /**
     * Retuns a value in m/s
     * 
     * @return
     */
    public double getAngleEncoderVelocitySI() {
        return angleMotor.getRotorVelocity().getValueAsDouble() // convert from RAW units, which are per 1s
                * Constants.SwerveChassis.TalonFXSwerveConfiguration.degreePerRotationFX;
    }

    /*
     * Returns the absolute position in degrees from rotations
     * The multiplication by 360 converts rotations to degs.
     * Adding 360 makes any negative degs into equivalent positive degs.
     * Mod 360 keeps any numbers over 360 to be reset to the equivalent value
     * between 0 and 360
     */
    private double getCancoderAbsEncoderValueDegrees() {
        return (cancoder.getAbsolutePosition().getValueAsDouble() * 360.0 + 360.0) % 360.0;
    }

    /**
     * Get Cancoder value in rotations; convert it, so always get positive number
     * @return
     */
    private double getCancoderAbsEncoderValue() {
        return (cancoder.getAbsolutePosition().getValueAsDouble() + 1.0) % 1.0;
    }

    public void setAngleMotorChassisAngleSI(double angle) {
        // angleMotor.set(TalonFXControlMode.MotionMagic, degreesToTicks(angle));
        angleMotor.setControl(new PositionDutyCycle(degreesToRotations(angle)));
    }

    public void applyPower(double power) {
        angleMotor.setControl(new DutyCycleOut(power));
    }

    private double degreesToRotations(double degrees) {
        return degrees / TalonFXSwerveConfiguration.degreePerRotationFX;
    }

    /**
     * Configure MotionMagic for Angle motor
     * 
     * @param c
     */
    private void configureMotionMagicAngle(Constants.SwerveChassis.SwerveModuleConstantsEnum c , TalonFXConfiguration talonFXConfigs) {

        // Disable motor safety so we can use hardware PID
        angleMotor.setSafetyEnabled(false);

        // angleMotor.configNeutralDeadband(FXAngle.NeutralDeadband, 30);

        // TODO: Recheck all of these parameters and set them for FALCON motors

        // angleMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,
        // FXAngle.periodMs, FXAngle.timeoutMs);
        // angleMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets,
        // FXAngle.periodMs, FXAngle.timeoutMs);

        // angleMotor.configPeakOutputForward(+1.0, FXAngle.timeoutMs);
        // angleMotor.configPeakOutputReverse(-1.0, FXAngle.timeoutMs);
        // angleMotor.configNominalOutputForward(0, FXAngle.timeoutMs);
        // angleMotor.configNominalOutputReverse(0, FXAngle.timeoutMs);

        /* FPID Gains */

        //var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = FXAngle.kS;
        slot0Configs.kP = FXAngle.kP;
        slot0Configs.kI = FXAngle.kI;
        slot0Configs.kD = FXAngle.kD;
        slot0Configs.kV = FXAngle.kV;
        slot0Configs.kA = FXAngle.kA;

        configureCurrentLimiterAngle(talonFXConfigs);

        // angleMotor.selectProfileSlot(FXAngle.SLOT_0, 0);
        // angleMotor.config_kP(FXAngle.SLOT_0, FXAngle.kP, FXAngle.timeoutMs);
        // angleMotor.config_kI(FXAngle.SLOT_0, FXAngle.kI, FXAngle.timeoutMs);
        // angleMotor.config_kD(FXAngle.SLOT_0, FXAngle.kD, FXAngle.timeoutMs);
        // angleMotor.config_kF(FXAngle.SLOT_0, FXAngle.kF, FXAngle.timeoutMs);

        // angleMotor.config_IntegralZone(FXAngle.SLOT_0, FXAngle.Izone,
        // FXAngle.timeoutMs);
        // angleMotor.configClosedLoopPeakOutput(FXAngle.SLOT_0, FXAngle.PeakOutput,
        // FXAngle.timeoutMs);
        // angleMotor.configAllowableClosedloopError(FXAngle.SLOT_0,
        // FXAngle.DefaultAcceptableError,
        // FXAngle.timeoutMs);

        // angleMotor.configClosedLoopPeriod(FXAngle.SLOT_0, FXAngle.closedLoopPeriod,
        // FXAngle.timeoutMs);

        // angleMotor.configMotionAcceleration(FXAngle.Acceleration, FXAngle.timeoutMs);
        // angleMotor.configMotionCruiseVelocity(FXAngle.CruiseVelocity,
        // FXAngle.timeoutMs);
        // angleMotor.configMotionSCurveStrength(FXAngle.Smoothing);

        //angleMotor.getConfigurator().apply(talonFXConfigs);
    }

    // Current limiter configuration for the angle motor
    private void configureCurrentLimiterAngle(TalonFXConfiguration tFxConfiguration) {
        // angleMotor.configSupplyCurrentLimit(
        // new SupplyCurrentLimitConfiguration(
        // TalonFXSwerveConfiguration.angleEnableCurrentLimit, // enable current limit
        // for drive motor
        // TalonFXSwerveConfiguration.angleContinuousCurrentLimit, // continuous draw to
        // drop to
        // TalonFXSwerveConfiguration.anglePeakCurrentLimit, // threshold after which
        // drop to continous limit
        // TalonFXSwerveConfiguration.anglePeakCurrentDuration
        // )
        // );
        // Not limiting OpenLoopRamp as we want the turn to run as fast as possible

        tFxConfiguration.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(
                                TalonFXSwerveConfiguration.anglePeakCurrentLimit)
                        .withStatorCurrentLimitEnable(
                                TalonFXSwerveConfiguration.angleEnableCurrentLimit));
    }

    // Current limiter configuration for the drive motor
    private void configureCurrentLimiterDrive(TalonFXConfiguration tFxConfiguration) {
        // driveMotor.configSupplyCurrentLimit(
        // new SupplyCurrentLimitConfiguration(
        // TalonFXSwerveConfiguration.driveEnableCurrentLimit, // enable current limit
        // for drive motor
        // TalonFXSwerveConfiguration.driveContinuousCurrentLimit, // continuous draw to
        // drop to
        // TalonFXSwerveConfiguration.drivePeakCurrentLimit, // threshold after which
        // drop to continous limit
        // TalonFXSwerveConfiguration.drivePeakCurrentDuration
        // )
        // );
        tFxConfiguration.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(
                                TalonFXSwerveConfiguration.drivePeakCurrentLimit)
                        .withStatorCurrentLimitEnable(
                                TalonFXSwerveConfiguration.driveEnableCurrentLimit));

        // driveMotor.configOpenloopRamp(
        // TalonFXSwerveConfiguration.driveSecondsFromNeutralToFull,
        // TalonFXSwerveConfiguration.configureTimeoutMs);

        tFxConfiguration.withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(
                        TalonFXSwerveConfiguration.driveSecondsFromNeutralToFull));
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
     * TalonFX rotates more than ones per rotation of the wheel because of the
     * gearbox.
     * Note that the wheels do not need to be set "forward" at the beginning of the
     * game. The absolute encoder in CANCODER
     * will set the right angle-related value to the relative encoder in TalonFX,
     * since
     * absolute encoders are not set to 0 after
     * power cycle. The drive routines will then change the wheel positions as
     * needed.
     */
    public void setEncoderforWheelCalibration(SwerveModuleConstantsEnum c, TalonFXConfiguration tFxC) {
        double encValue = getCancoderAbsEncoderValue(); // current absolute encodervalue
        double difference = encValue - (c.getAngleOffset()/360.0); // cancoder Method returns Abs value in Rotations
        double mEncValue = getAngleEncoderPosition();

        /*
         * "difference" added to current angle encoder RAW value should add to a whole number
         * That would mean that wheels pointing "straight" would have the whole number of rotations on the encoder.
         * The actual number does not matter, as long as it's a whole integer.
         * In Phonenix 6 the reporting adjustment to the encoder can only be a fraction between 0 an 1, so
         * we cannot zero the encoder, like we can in Phoenix 5. But we do not really need to.
         * Since we reset the TalonFX to factory settings every time we start the robot, our adjustment is done every time as well.
         */

        double adjustment = ((mEncValue-difference) % 1.0) ;

        // alex test
        // System.out.println(c.getAngleMotorID()
        //     + " CAE: " + getCancoderAbsEncoderValue()
        //     + " AO: " + c.getAngleOffset()/360.0 
        //     + " AEP: " + mEncValue
        //     + " D: " + difference
        //     + " A: " + adjustment);

        //if (difference < 0) {
            // difference += TalonFXSwerveConfiguration.clicksFXPerFullRotation;
        //    difference += 1.0;
        //}

        // if (difference <= TalonFXSwerveConfiguration.clicksFXPerFullRotation / 2) {
        // encoderSetting = difference;

        // } else {
        // encoderSetting = difference -
        // TalonFXSwerveConfiguration.clicksFXPerFullRotation;
        // }

        //if (difference <= 1.0 / 2.0) {
        //    encoderSetting = difference;

        //} else {
        //    encoderSetting = difference - 1.0;
        //}

        // angleMotor.setSelectedSensorPosition(encoderSetting);
        //TalonFXConfiguration tFxC = new TalonFXConfiguration();
        System.out.println("AF: " + c.getAngleMotorID() + " " + -adjustment);

        //tFxC.Feedback.withFeedbackRotorOffset(encoderSetting / 360.0);
        // angleMotor.getConfigurator().apply(tFxC);

        tFxC.Feedback.FeedbackRotorOffset = adjustment ;

        System.out.println("Set encoder adjustment for motor " + c.getAngleMotorID() + " to " + adjustment);

    }

    private void driveMotorBrakeMode() {
        TalonFXConfigurator tFxCr = driveMotor.getConfigurator();
        TalonFXConfiguration tFxC = new TalonFXConfiguration() ;
        tFxCr.refresh(tFxC);
        tFxC.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        driveMotor.getConfigurator().apply(tFxC);
        // driveMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void angleMotorBrakeMode() {
        TalonFXConfigurator tFxCr = angleMotor.getConfigurator();
        TalonFXConfiguration tFxC = new TalonFXConfiguration();
        tFxCr.refresh(tFxC);
        tFxC.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        angleMotor.getConfigurator().apply(tFxC);
        // angleMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void driveMotorCoastMode() {
        TalonFXConfigurator tFxCr = driveMotor.getConfigurator();
        TalonFXConfiguration tFxC = new TalonFXConfiguration();
        tFxCr.refresh(tFxC);
        tFxC.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        driveMotor.getConfigurator().apply(tFxC);
        // driveMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void angleMotorCoastMode() {
        TalonFXConfigurator tFxCr = angleMotor.getConfigurator();
        TalonFXConfiguration tFxC = new TalonFXConfiguration();
        tFxCr.refresh(tFxC);
        tFxC.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        angleMotor.getConfigurator().apply(tFxC);
        // angleMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {

        // While we update the current angle of the angle motor for telemetry,
        // It's not used in the teleop driving, as we use real-time update via getState
        // call.
        currentAngle = Rotation2d.fromDegrees(getAngleEncoderPositionSI());

        // integratedAngleEncoder.setPosition(cancoder.getAbsolutePosition() -
        // angleOffset);
    }
}
