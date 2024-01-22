// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveChassis;

/**
 * Runs trajectory. The command will not update initial odometry of the robot.
 * That should be done by a separate command preceding this one.
 */
public class AutonomousTrajectoryRioCommand extends FollowPathHolonomic {
  /** Creates a new AutonomousTrajectoryRioCommand.
   * This command runs the trajectory using PathPlanner holonomic trajectory controller.
   * The trajectory provided to this command must be in a form of PathPlannerTrajectory object.
   * That means that any adjustments, such as reducing its speed, reversal etc must be
   * applied before the trajectory is passed to this command.
   * Note that PathPlanner ends trajectories on time rather that completion of the distance.
   * Therefore, PID constants provided to the PID controller may have an impact on the end result,
   * especially for the holonomic component.
   */

  TrajectoryConfig config;

  PathPlannerPath trajectoryPath;
  
  public AutonomousTrajectoryRioCommand(PathPlannerPath trajectoryPath) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    
    // Since we extend PPSwerveControllerCommand, we need to call its constructor properly
    // This command was done to provide better control and telemetry over the execution of
    // the PathPlanner trajectory
    super(
      trajectoryPath, //path
      RobotContainer.driveSubsystem::getPose, //supplier of pose2d
      RobotContainer.driveSubsystem::getChassisSpeeds, //supplier of max chassis speeds
      RobotContainer.driveSubsystem::setChassisSpeeds,  //consumer of chassis speeds
      new PIDConstants(SwerveChassis.DRIVE_CHASSIS_KP,  // Translation PID constants
                        SwerveChassis.DRIVE_CHASSIS_KI,
                        SwerveChassis.DRIVE_CHASSIS_KD),
      new PIDConstants(SwerveChassis.ANGLE_CHASSIS_KP,  // Rotation PID constants
                        SwerveChassis.ANGLE_CHASSIS_KI,
                        SwerveChassis.ANGLE_CHASSIS_KD),  //both PID constants
      SwerveChassis.MAX_VELOCITY, //Max module speed
      SwerveChassis.CHASSIS_OUTER_DRIVE_RADIUS, //drive base radius
      new ReplanningConfig(), //replanning config
      () -> false,  //mirror trajectory - change when needed
      RobotContainer.driveSubsystem //subsystem dependency
    );
    this.trajectoryPath = trajectoryPath;
  }

  // Run trajectory with known maximum velocity and acceleration
  /**
   * @param trajectoryName Filename containing trajectory without .path
   */
  public AutonomousTrajectoryRioCommand(String trajectoryName){
    this(PathPlannerPath.fromPathFile(trajectoryName));
    System.out.println("initalized trajectory: " + trajectoryName);
  }

  public ChassisSpeeds getChassisSpeeds() {
   return new ChassisSpeeds(SwerveChassis.MAX_VELOCITY, SwerveChassis.MAX_VELOCITY, SwerveChassis.MAX_ANGULAR_VELOCITY);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    System.out.println("Auto trajectory initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update robot odometry

    //System.out.println("O");

    /**
     * In this example project we only update field odometry when needed, meaning when running
     * automated trajectories. However, you may need to update it in other situations, especially
     * when using vision to determine robot's position on the field.
     */
    RobotContainer.driveSubsystem.updateTrajectoryOdometry();

    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("*** End trajectory command. Interrupted:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return super.isFinished();

  }
}
