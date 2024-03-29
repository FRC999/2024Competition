package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveChassis;

public class TurnToRelativeAngleSoftwarePIDCommand extends Command {

	// Software PID turn constants
	private final double kP = 0.15958;
	private final double kI = 0.02;
	private final double kD = 0.0;
	private final double minOmega = 0.37;
	Rotation2d angle;
	Supplier<Rotation2d> angleSupplier;
	private double kMaxSpeed = SwerveChassis.MAX_ANGULAR_VELOCITY; // radians per second
	private double kMaxAccel = Math.PI * 16; // radians per second square
	//private double kMaxSpeed = 360;
	//private double kMaxAccel = 720;
	private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAccel);
	private PIDController profiledPID = new PIDController(kP, kI, kD);
	private double tolerance = 0.5; // degrees of tolerance to end the command
	private double finalGoal = 0.0;

	/**
	 * This class is used to rotate the robot to RELATIVE dynamic angle using software PID
	 * using drive() method of the DriveSubsystem
	 * It may be used in case PP does not do clean in-place rotation
	 * @param angle
	 */
	public TurnToRelativeAngleSoftwarePIDCommand(Supplier<Rotation2d> a) {
		angleSupplier = a;
		addRequirements(RobotContainer.driveSubsystem);
		//profiledPID.enableContinuousInput(0, 360);
		profiledPID.disableContinuousInput();
		profiledPID.setTolerance(tolerance);
	}

	@Override
	public void initialize() {
		angle = angleSupplier.get();
		finalGoal = RobotContainer.imuSubsystem.getYaw()+angle.getDegrees();
		profiledPID.reset();
		profiledPID.setSetpoint(finalGoal);
	}

	@Override
	public void execute() {
		double omegaDegPerSec = MathUtil.clamp(profiledPID.calculate(RobotContainer.imuSubsystem.getYaw()),-0.85,0.85);
		//System.out.println("O1:"+omegaDegPerSec);
		omegaDegPerSec = (omegaDegPerSec<0)? omegaDegPerSec - minOmega :  omegaDegPerSec + minOmega ;
		//System.out.println("******yaw: " + RobotContainer.imuSubsystem.getYaw() + " a: " + angle.getDegrees() + " g: " +finalGoal + " o: " + Units.degreesToRadians(omegaDegPerSec)/ SwerveChassis.MAX_ANGULAR_VELOCITY);
		RobotContainer.driveSubsystem.drive(0, 0, omegaDegPerSec, true);
		//profiledPID.setGoal(RobotContainer.imuSubsystem.getYaw()+angle.getDegrees());  // get new YAW
		//System.out.println("O2:"+omegaDegPerSec);
	}

	@Override
	public void end(boolean interrupted) {
	  super.end(interrupted);
	  System.out.println("*** End turn command. Interrupted:"+interrupted);
	}
  
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
  	  return profiledPID.atSetpoint() ;
  	}
}