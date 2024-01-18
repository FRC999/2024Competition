package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.Constants.SwerveChassis.SwerveTelemetry;



public class SwerveModule{

    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX angleMotor;

    


        private int moduleNumber;
        private double angleOffset;
        
        private double lastAngle;

    private Rotation2d currentAngle = new Rotation2d();

    // This is not used for the manual teleop driving
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveChassis.DRIVE_KS,
            Constants.SwerveChassis.DRIVE_KV, Constants.SwerveChassis.DRIVE_KA);

    /**
     * This subsystem instantiates and manages a single swerve module.
     * Since you have multiple swerve modules, this class is probably instantiated in a loop.
     * It provides logic for the swerve operation including state changes.
     * It is defined as a Subsystem in case you want to have a periodic code in it.
     * This class should instantiate all relevant motors. Since relevant encoders are only
     * used for driving, they're exposed via corresponding motor objects, rather than separate objects.
     * However, since IMU may be used for other functions besides driving, IMU is exposed via
     * a separate subsystem, instantiated in RobotContainer via public static final object.
     * Hence, all IMU-related calls need to use that object.
     * 
     * @param moduleNumber - identifies module numerically (e.g. 0,1,2,3) - primarily used for logging
     * @param moduleConstants - supplies module configuration parameters via enum defined in Constants
     */
    
    
        public SwerveModule(int moduleNumber, SwerveModuleConstantsEnum moduleConstants) {
            this.moduleNumber = moduleNumber;
            angleOffset = moduleConstants.getAngleOffset();
    
            // Assuming Motor is a class/interface representing your motor control
            driveMotor = new TalonSRXMotor(moduleConstants.getDriveMotorID());
            angleMotor = new TalonSRXMotor(moduleConstants.getAngleMotorID());
    
            // Configure motors as needed
            driveMotor.configureDriveMotor(moduleConstants);
            angleMotor.configureAngleMotor(moduleConstants);
    
            lastAngle = getState().angle.getDegrees();
        }
        
        // Additional methods and logic for the SwerveModule class
    }
    

    public SwerveModuleState getState() {
        double velocity = driveMotor.getDriveEncoderVelocitySI();
        Rotation2d angle = Rotation2d.fromDegrees(angleMotor.getAngleEncoderPositionSI());
        return new SwerveModuleState(velocity, angle);
    }

    public double telemetryAngleEncoder(){
        return angleMotor.getAngleEncoderPosition();
    }

    public double telemetryAngleEncoderSI(){
        return angleMotor.getAngleEncoderPositionSI();
    }

    public double telemetryDriveEncoder(){
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

    public int getModuleNumber(){
        return moduleNumber;
    }

    
    public void setDesiredState(SwerveModuleState desiredState) {

        // Minimizes angle movement of angle motor by limiting movement to 90 degrees and 
        // reversing power to negative value if necessary
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // Use this flag for chassis testing, if you want to see the angle and power numbers provided by the Swerve calculations
        // instead of actually moving the robot

        switch (SwerveTelemetry.swerveDriveOrTelemetry) {
            case DRIVE_ONLY:
                 // This is the code that makes the robot move by applying the power to the motors
                driveMotor.applyPower(desiredState.speedMetersPerSecond / SwerveChassis.MAX_VELOCITY);
                angleMotor.setAngleMotorChassisAngleSI(desiredState.angle.getDegrees()); // Rotation2d angle does not give degrees
                break;
            case TELEMETRY_ONLY:
                printSwerveModuleState(desiredState);
                break;
            case DRIVE_AND_TELEMETRY:
                printSwerveModuleState(desiredState);
                // This is the code that makes the robot move by applying the power to the motors
                driveMotor.applyPower(desiredState.speedMetersPerSecond / SwerveChassis.MAX_VELOCITY);
                angleMotor.setAngleMotorChassisAngleSI(desiredState.angle.getDegrees()); // Rotation2d angle does not give degrees
                break;
        }            
    }

    public void setDesiredStateCalibration(SwerveModuleState desiredState) {

        // Minimizes angle movement of angle motor by limiting movement to 90 degrees and 
        // reversing power to negative value if necessary
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // Use this flag for chassis testing, if you want to see the angle and power numbers provided by the Swerve calculations
        // instead of actually moving the robot
        printSwerveModuleState(desiredState);
    }

    public void printSwerveModuleState(SwerveModuleState moduleState) {
        System.out.print(" SM: "+moduleNumber);
        System.out.print(" P: "+moduleState.speedMetersPerSecond / SwerveChassis.MAX_VELOCITY);
        System.out.println(" A: "+moduleState.angle.getDegrees());
    }

    //TODO: This position is currently set in the encoder units. This may need to change to the SI units. Investigate.
    /**
     * Returns a robot-centric position of the swerve module
     * It is only used in odometry calculations, meaning, is only used for automated/trajectory driving
     * and not for teleop/manual driving.
     * @return SwerveModulePosition - WPILIB kinematics object 
     */
    public SwerveModulePosition getPosition() {
        double position = driveMotor.getDriveEncoderPositionSI(); 
        Rotation2d angle = currentAngle;
        return new SwerveModulePosition(position, angle);
    }

    @Override
    public void periodic() {

        // While we update the current angle of the angle motor for telemetry,
        // It's not used in the teleop driving, as we use real-time update via getState call.
        currentAngle = Rotation2d.fromDegrees(angleMotor.getAngleEncoderPositionSI());

        //integratedAngleEncoder.setPosition(cancoder.getAbsolutePosition() - angleOffset);
    }
}
