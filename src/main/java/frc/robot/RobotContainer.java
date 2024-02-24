// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveChassis.SwerveTelemetry;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.commands.ArmStop;
import frc.robot.commands.AutonomousTrajectory2Poses;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.PosePrinter;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.AutonomousTrajectory2PosesDynamic;
import frc.robot.commands.CalibrateArmPowerFF;
import frc.robot.commands.CalibrateIntakePower;
import frc.robot.commands.CalibrateShooterPower;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbStop;
import frc.robot.commands.ClimbUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LLDetectorSubsystem;
import frc.robot.subsystems.LLVisionSubsystem;
import frc.robot.subsystems.NetworkTablesSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  public final static IMUSubsystem imuSubsystem = new IMUSubsystem();
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  //public final static SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  public final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final static ArmSubsystem armSubsystem = new ArmSubsystem();
  public final NetworkTablesSubsystem networkTableSubsystem = new NetworkTablesSubsystem();
  public final static ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  public final static LLVisionSubsystem llVisionSubsystem = new LLVisionSubsystem();
  public final static PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem(PhotonVisionConstants.PVCameraName);
  public final static LLDetectorSubsystem llDetectorSubsystem = new LLDetectorSubsystem();
  public final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
  
  public static Controller xboxController;

  public static Controller driveStick; // for robot testing only
  public static Controller driveStick1; // for robot testing only
  public static Controller driveStick2; // for robot testing only
  public static Controller driveStick3; // for robot testing only

  // A Data Log Manager file handle
  public static StringLogEntry myStringLog;

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

      // Calibrate-only joysticks
      if (DebugTelemetrySubsystems.calibrateShooter)
        driveStick1 = new Controller(ControllerDevice.DRIVESTICK1);
      if (DebugTelemetrySubsystems.calibrateIntake)
        driveStick2 = new Controller(ControllerDevice.DRIVESTICK2);
      if (DebugTelemetrySubsystems.calibrateArm)
        driveStick3 = new Controller(ControllerDevice.DRIVESTICK3);

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

    helperButtons();

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

    // Test PathPlanner - DONE
    // trajectoryCalibration();

    // Test Vision
    // visionTesting();

    // ============================
    // === Calibration bindings ===
    // ============================

    // Calibrate intake
    if (DebugTelemetrySubsystems.calibrateIntake) {
        calibrateIntakePower();
    }

    // Calibrate shooter
    if (DebugTelemetrySubsystems.calibrateShooter) {
        calibrateShooterPower();
    }

    // Calibrate arm
    if (DebugTelemetrySubsystems.calibrateArm) {
        calibrateArmPowerFF();
    }
  }

  // Driver preferred controls
  private double getDriverXAxis() {
    //return -xboxController.getLeftStickY();
    return -xboxController.getRightStickY();
  }

  private double getDriverYAxis() {
    //return -xboxController.getLeftStickX();
    return -xboxController.getRightStickX();
  }

  private double getDriverOmegaAxis() {
    //return -xboxController.getLeftStickOmega();
    return xboxController.getLeftStickX();
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

  public void visionTesting() {

    // LL Vision and shooting pose checks

    // Blue Amp Shooting
    new JoystickButton(driveStick, 1)
        .onTrue(
            new PrintCommand("Blue AMP - driving from\n").andThen(
                new PosePrinter(llVisionSubsystem::getRobotFieldPoseLL)).andThen(
                    new PrintCommand("\n to \n" +
                        VisionConstants.blueAmpShootingPose)))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

    // Blue Speaker Shooting
    new JoystickButton(driveStick, 2)
        .onTrue(
            new PrintCommand("Blue Speaker - driving from\n").andThen(
                new PosePrinter(llVisionSubsystem::getRobotFieldPoseLL)).andThen(
                    new PrintCommand("\n to \n" +
                        VisionConstants.blueSpeakerShootingPose)))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

    // Red Speaker Shooting
    new JoystickButton(driveStick, 3)
        .onTrue(
            new PrintCommand("Red Speaker - driving from\n").andThen(
                new PosePrinter(llVisionSubsystem::getRobotFieldPoseLL)).andThen(
                    new PrintCommand("\n to \n" +
                        VisionConstants.redSpeakerShootingPose)))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

    // *** === Commands that actually drive; caution; check the FROM/TO poses before
    // trying them ===
    // **** MAKE SURE to check the prinout of the poses first (see triggers above)
    // **** CAUTION !!! DO NOT run unless you have ample space to test

    // Blue Amp driving

    // new JoystickButton(driveStick, 7)
    // .whileTrue(new AutonomousTrajectory2Poses(() ->
    // llVisionSubsystem.getRobotFieldPoseLL(), () ->
    // VisionConstants.blueAmpShootingPose))
    // .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot,
    // RobotContainer.driveSubsystem));

    // red Speaker Shooting
    new JoystickButton(driveStick, 8)
        // Test with relatively slow max velocity and acceleration
        .whileTrue(new AutonomousTrajectory2PosesDynamic(() -> llVisionSubsystem.getRobotFieldPoseLL(),
            () -> VisionConstants.redSpeakerShootingPose))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

  }

  /**
   * Testing of the routines related to the note detection
   */
  public void visionDetectorTesting() {

  }

  public void trajectoryCalibration() {
    new JoystickButton(driveStick, 1)
        .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("2023OneMeterForward"))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
    new JoystickButton(driveStick, 2)
        .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("2023OneMeterBackwards"))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
    new JoystickButton(driveStick, 3)
        .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("2023OneMeter45"))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
    new JoystickButton(driveStick, 4)
        .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("2023ThreeMeterForward"))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
    new JoystickButton(driveStick, 5)
        .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("2023ThreeMeterForward90"))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

    new JoystickButton(driveStick, 6)
        .whileTrue(new AutonomousTrajectory2Poses(testPoseSupplier(1, 1, 0), testPoseSupplier(2, 1, 0)))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

    new JoystickButton(driveStick, 7)
        .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("2023ThreeMeterRight"))
        .onFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));

  }

  public void testIntake() {
      new JoystickButton(driveStick, 3)
              .whileTrue(new InstantCommand(() -> RobotContainer.intakeSubsystem.runIntake(0.2)))
              .onFalse(new InstantCommand(RobotContainer.intakeSubsystem::stopIntake));
  }

  public void testArm() {
      new JoystickButton(driveStick, 3)
              .whileTrue(new InstantCommand(() -> RobotContainer.armSubsystem.runArmMotors(0.2)))
              .onFalse(new InstantCommand(RobotContainer.armSubsystem::stopArmMotors));
      new JoystickButton(driveStick, 4)
              .whileTrue(new InstantCommand(() -> RobotContainer.armSubsystem.runArmMotors(-0.2)))
              .onFalse(new InstantCommand(RobotContainer.armSubsystem::stopArmMotors));
  }

  public void testClimber() {
      new JoystickButton(driveStick, 3)
              .whileTrue(new ClimbUp())
              .onFalse(new ClimbStop());
      new JoystickButton(driveStick, 4)
              .whileTrue(new ClimbDown())
              .onFalse(new ClimbStop());
  }

  Pose2d testPoseSupplier(double x, double y, double angle) {
      return new Pose2d(x, y, new Rotation2d().fromDegrees(angle));
  }

  public void helperButtons() {
      new JoystickButton(driveStick, 12)
              .onTrue(new InstantCommand(RobotContainer.imuSubsystem::zeroYaw));

  }

  // GPM Calibration

  public void calibrateShooterPower() {
      new JoystickButton(driveStick1, 1)
              .whileTrue(new CalibrateShooterPower())
              .onFalse(new ShooterStop());
  }

  public void calibrateIntakePower() {
      new JoystickButton(driveStick2, 1)
              .whileTrue(new CalibrateIntakePower())
              .onFalse(new IntakeStop());
  }

  public void calibrateArmPowerFF() {
      new JoystickButton(driveStick3, 1)
              .whileTrue(new CalibrateArmPowerFF())
              .onFalse(new ArmStop());
  }



  public void checkAllianceColor() {
    SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
  }

}
