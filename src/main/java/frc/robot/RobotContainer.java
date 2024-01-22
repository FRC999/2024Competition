// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveChassis.SwerveTelemetry;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GPMSubsystem;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.NetworkTablesSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static IMUSubsystem imuSubsystem = new IMUSubsystem();
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  //public final GPMSubsystem gpmSubsystem = new GPMSubsystem();
  public final NetworkTablesSubsystem networkTableSubsystem = new NetworkTablesSubsystem();
  public static Controller xboxController;

  public static Controller driveStick; // for robot testing only

  // A Data Log Manager file handle
  public static StringLogEntry myStringLog;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (SwerveTelemetry.saveDataUsingDataLogManager) {
      DataLogManager.start();
      DataLog log = DataLogManager.getLog();
      myStringLog = new StringLogEntry(log, SwerveTelemetry.logFileName);
    }



    // Configure driver interface - binding joystick objects to port numbers
    configureDriverInterface();

    // Configure the trigger bindings
    configureBindings();

    driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverXAxis(),
            () -> getDriverYAxis(),
            () -> getDriverOmegaAxis(),
            () -> getDriverFieldCentric()));

      
  }

  /**
   * Use this method to define your controllers depending on the
   * {@link DriveInterface}
   */
  private void configureDriverInterface() {

      /**
       * We tried driving with a single Logitech joystick that has X,Y and turn axis.
       * However, the team decided to move the turn to the second joystick for now.
       * Note that Controller objects are only used to provide DoubleSupplier methods
       * to the
       * commands that need manual control input (e.g. DriveManuallyCommand)
       */
      driveStick = new Controller(ControllerDevice.DRIVESTICK);  // disable joysticks for driver practice code
      //turnStick = new Controller(ControllerDevice.TURNSTICK);   // disable joysticks for driver practice code
      xboxController = new Controller(ControllerDevice.XBOX_CONTROLLER);
      // bbl = new Joystick(OIConstants.bblPort);
      // bbr = new Joystick(OIConstants.bbrPort);

      System.out.println("Driver interface configured");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());


    // Test Motor working, Inversion, and Encder phase - DONE
    // testCalibrateMotorsAndEncodersButtonBindings();

    // Test Swerve drive routines - DONE
    // swerveValuesTesting();

    // Test PathPlanner
    trajectoryCalibration();

  }

  private double getDriverXAxis() {
    return -xboxController.getLeftStickY();
  }

  private double getDriverYAxis() {
    return -xboxController.getLeftStickX();
  }

  private double getDriverOmegaAxis() {
    return -xboxController.getLeftStickOmega();
  }

    /**
   * If the button is pressed, use robot-centric swerve
   * Otherwise use field-centric swerve (default).
   * Currently it's set to a numbered button on a joystick, but if you use Xbox or
   * similar controller, you may need to modify this
   * On Logitech joystick button #2 seemed to be the most convenient, though we
   * may consider moving it to the drivestick.
   * 
   * @return - true if robot-centric swerve should be used
   */
  private boolean getDriverFieldCentric() {
      return !xboxController.getRawButton(OIConstants.robotCentricButton);
  }

  /**
   * #######################################################
   * Methods used for motor, chassis and auto testing
   * #######################################################
   */

  /**
   * 
  * Make sure motors move robot forward with positive power and encoders increase with positive power
  * To enable put a call to this method in configureBindings method
  */
  @SuppressWarnings("unused")
  private void testCalibrateMotorsAndEncodersButtonBindings() {

    new JoystickButton(driveStick, 1)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testDriveMotorEncoderPhase(0)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)));

    new JoystickButton(driveStick, 3)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testDriveMotorEncoderPhase(1)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)));

    new JoystickButton(driveStick, 5)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testDriveMotorEncoderPhase(2)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)));

    new JoystickButton(driveStick, 7)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testDriveMotorEncoderPhase(3)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)));

    new JoystickButton(driveStick, 2)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testAngleMotorEncoderPhase(0)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0)));

    new JoystickButton(driveStick, 4)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testAngleMotorEncoderPhase(1)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)));

    new JoystickButton(driveStick, 6)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testAngleMotorEncoderPhase(2)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)));

    new JoystickButton(driveStick, 8)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.testAngleMotorEncoderPhase(3)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)));
  }

  @SuppressWarnings("unused")
  private void swerveValuesTesting() { // Field centric numbers applied

    // Move robot to the left
    new JoystickButton(driveStick, 3)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0,3,0, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Move robot forward
    new JoystickButton(driveStick, 1)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(3,0,0, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Move robot to the right
    new JoystickButton(driveStick, 4)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0,-3,0, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Move robot backwards
    new JoystickButton(driveStick, 2)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(-3,0,0, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Turn robot counterclockwise
    new JoystickButton(driveStick, 7)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0,0, 10, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );

    // Turn robot clockwise
    new JoystickButton(driveStick, 6)
        .whileTrue(new InstantCommand(() -> RobotContainer.driveSubsystem.drive(0,0, -10, true)))
        .whileFalse(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(0))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopAngleMotor(3)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(0)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(1)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(2)))
           .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.stopDriveMotor(3)))
        );
  }

  @SuppressWarnings("unused")
  public void trajectoryCalibration() {
      new JoystickButton(driveStick, 1)
              .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("OneMeterForward"))
              .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
