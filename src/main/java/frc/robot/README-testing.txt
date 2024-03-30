Current items to test:

=== AutoCBlueMid4NotesOptimized

===AutoCRedMid4NotesOptimized


=== ArmDownToSwitch  (Pigeon reset)
    new Trigger(() -> (xboxDriveController.getRawButton(5) && (xboxDriveController.getRawButtonPressed(7))))
        .onTrue(new ArmDownToSwitch())
        .onFalse(new ArmStop().andThen(new IMUReset()));



=== Check if BOTH Arm NEOs are working (suspiciously 0 power on the second one)



=== Update LL version - may be (it seems to work now)

=== Automatic teleop node pickup

#####################
###### IN PROGRESS
#####################

--- tested with Software PID. Works. However, may be easily affected by the motors; more testing will be needed
=== ShootUsingLLAndTurn

#####################
###### TESTED/DONE
#####################

--- TESTED; DOES NOT WORK; NEED SOFTWARE PID due to the very fine control required
=== Turning via PP
    new JoystickButton(driveStick,9)
        .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("2023-90-turn",1.0,0.2, false))
        .onFalse(new StopAllMotorsCommand());

--- TESTED. Works
=== Green LED ON when in shooting distance and see AT
    new Trigger(() -> TrajectoryHelpers.isValueBetween(llVisionSubsystem.getShootingDistance(), 0.0, 3.5 ))
        .onTrue(
            new InstantCommand(RobotContainer.candleSubsystem::setLEDGreen).onlyIf(()->EnabledSubsystems.candle)
        )

--- TESTED. Works
== Blue LED ON when turned to Speaker
    new Trigger(() -> TrajectoryHelpers.isValueBetween(llVisionSubsystem.getShootingDistance(), 0.0, 3.5 ) 
                    && Math.abs(llVisionSubsystem.getRotationAngleToSpeaker().getDegrees())<3)
        .onTrue(
            new InstantCommand(RobotContainer.candleSubsystem::setLEDAngle).onlyIf(()->EnabledSubsystems.candle)
        )
        
--- No need to update firmware, Updated libraries. 
=== Update firmware on NEOs if needed
