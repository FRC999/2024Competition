package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.Constants.SwerveChassis.SwerveTelemetry;

public class SwerveModule {

    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX angleMotor;

    private int moduleNumber;
    private double angleOffset;
    private double lastAngle;

    private Rotation2d currentAngle = new Rotation2d();

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveChassis.DRIVE_KS,
            Constants.SwerveChassis.DRIVE_KV, Constants.SwerveChassis.DRIVE_KA);

 
    public SwerveModule(int moduleNumber, SwerveModuleConstantsEnum moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.getAngleOffset();

        driveMotor = new TalonSRXMotor(moduleConstants.getDriveMotorID());
        angleMotor = new TalonSRXMotor(moduleConstants.getAngleMotorID());

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

}

