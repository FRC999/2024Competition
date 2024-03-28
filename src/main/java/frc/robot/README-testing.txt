Current items to test:

AutoCBlueMid4NotesOptimized
AutoCRedMid4NotesOptimized

ShootUsingLLAndTurn

ArmDownToSwitch  (Pigeon reset)
    new Trigger(() -> (xboxDriveController.getRawButton(5) && (xboxDriveController.getRawButtonPressed(7))))
        .onTrue(new ArmDownToSwitch())
        .onFalse(new ArmStop().andThen(new IMUReset()));

Turning via PP
    new JoystickButton(driveStick,9)
        .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("2023-90-turn",1.0,0.2, false))
        .onFalse(new StopAllMotorsCommand());