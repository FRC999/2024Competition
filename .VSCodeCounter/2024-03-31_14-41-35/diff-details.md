# Diff Details

Date : 2024-03-31 14:41:35

Directory t:\\Projects\\MechaRAMS\\2024Competition\\src\\main

Total : 38 files,  -362 codes, 172 comments, 99 blanks, all -91 lines

[Summary](results.md) / [Details](details.md) / [Diff Summary](diff.md) / Diff Details

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [.pathplanner/settings.json](/.pathplanner/settings.json) | JSON with Comments | -12 | 0 | 0 | -12 |
| [.wpilib/wpilib_preferences.json](/.wpilib/wpilib_preferences.json) | JSON | -6 | 0 | 0 | -6 |
| [README.md](/README.md) | Markdown | -1 | 0 | 0 | -1 |
| [WPILib-License.md](/WPILib-License.md) | Markdown | -22 | 0 | -3 | -25 |
| [build.gradle](/build.gradle) | Groovy | -62 | -19 | -21 | -102 |
| [gradle/wrapper/gradle-wrapper.properties](/gradle/wrapper/gradle-wrapper.properties) | Java Properties | -7 | 0 | -1 | -8 |
| [gradlew.bat](/gradlew.bat) | Batch | -41 | -30 | -22 | -93 |
| [settings.gradle](/settings.gradle) | Groovy | -28 | 0 | -3 | -31 |
| [src/main/java/frc/robot/Constants.java](/src/main/java/frc/robot/Constants.java) | Java | 21 | 64 | 8 | 93 |
| [src/main/java/frc/robot/Robot.java](/src/main/java/frc/robot/Robot.java) | Java | 1 | -1 | 0 | 0 |
| [src/main/java/frc/robot/RobotContainer.java](/src/main/java/frc/robot/RobotContainer.java) | Java | 30 | 20 | 15 | 65 |
| [src/main/java/frc/robot/commands/ArmDownToNoteVisionForAutoNotePickup.java](/src/main/java/frc/robot/commands/ArmDownToNoteVisionForAutoNotePickup.java) | Java | 10 | 9 | 5 | 24 |
| [src/main/java/frc/robot/commands/ArmDownToSwitch.java](/src/main/java/frc/robot/commands/ArmDownToSwitch.java) | Java | 25 | 9 | 8 | 42 |
| [src/main/java/frc/robot/commands/AutoBlueHigher2CalibrationCorrected.java](/src/main/java/frc/robot/commands/AutoBlueHigher2CalibrationCorrected.java) | Java | 51 | 38 | 21 | 110 |
| [src/main/java/frc/robot/commands/AutoCBlueMid4NotesOptimized.java](/src/main/java/frc/robot/commands/AutoCBlueMid4NotesOptimized.java) | Java | 97 | 19 | 27 | 143 |
| [src/main/java/frc/robot/commands/AutoCRNCBlue2CenterFromBottom.java](/src/main/java/frc/robot/commands/AutoCRNCBlue2CenterFromBottom.java) | Java | 4 | 6 | 2 | 12 |
| [src/main/java/frc/robot/commands/AutoCRNCRed2CenterFromBottom.java](/src/main/java/frc/robot/commands/AutoCRNCRed2CenterFromBottom.java) | Java | 1 | 1 | 0 | 2 |
| [src/main/java/frc/robot/commands/AutoCRedMid4NotesOptimized.java](/src/main/java/frc/robot/commands/AutoCRedMid4NotesOptimized.java) | Java | 97 | 19 | 27 | 143 |
| [src/main/java/frc/robot/commands/ControllerRumble.java](/src/main/java/frc/robot/commands/ControllerRumble.java) | Java | 12 | 8 | 5 | 25 |
| [src/main/java/frc/robot/commands/IMUReset.java](/src/main/java/frc/robot/commands/IMUReset.java) | Java | 11 | 8 | 5 | 24 |
| [src/main/java/frc/robot/commands/IntakeGrabNote.java](/src/main/java/frc/robot/commands/IntakeGrabNote.java) | Java | 7 | 1 | 0 | 8 |
| [src/main/java/frc/robot/commands/NotePickupCamera.java](/src/main/java/frc/robot/commands/NotePickupCamera.java) | Java | 46 | 10 | 12 | 68 |
| [src/main/java/frc/robot/commands/ShootUsingLLAndTurn.java](/src/main/java/frc/robot/commands/ShootUsingLLAndTurn.java) | Java | -6 | -31 | -12 | -49 |
| [src/main/java/frc/robot/commands/StopChassis.java](/src/main/java/frc/robot/commands/StopChassis.java) | Java | 15 | 9 | 6 | 30 |
| [src/main/java/frc/robot/commands/TurnToRelativeAngleSoftwarePIDCommand.java](/src/main/java/frc/robot/commands/TurnToRelativeAngleSoftwarePIDCommand.java) | Java | 7 | 5 | 0 | 12 |
| [src/main/java/frc/robot/lib/GPMHelpers.java](/src/main/java/frc/robot/lib/GPMHelpers.java) | Java | 8 | 6 | 2 | 16 |
| [src/main/java/frc/robot/lib/TrajectoryHelpers.java](/src/main/java/frc/robot/lib/TrajectoryHelpers.java) | Java | 9 | 4 | 4 | 17 |
| [src/main/java/frc/robot/subsystems/ArmSubsystem.java](/src/main/java/frc/robot/subsystems/ArmSubsystem.java) | Java | 8 | 5 | 2 | 15 |
| [src/main/java/frc/robot/subsystems/CANdleSubsystem.java](/src/main/java/frc/robot/subsystems/CANdleSubsystem.java) | Java | 21 | 0 | 3 | 24 |
| [src/main/java/frc/robot/subsystems/DriveSubsystem.java](/src/main/java/frc/robot/subsystems/DriveSubsystem.java) | Java | 0 | 2 | 1 | 3 |
| [src/main/java/frc/robot/subsystems/LLVisionSubsystem.java](/src/main/java/frc/robot/subsystems/LLVisionSubsystem.java) | Java | 21 | 7 | 3 | 31 |
| [src/main/java/frc/robot/subsystems/PhotonVisionNoteHuntingSubsystem.java](/src/main/java/frc/robot/subsystems/PhotonVisionNoteHuntingSubsystem.java) | Java | 16 | 3 | 6 | 25 |
| [vendordeps/PathplannerLib2023.json](/vendordeps/PathplannerLib2023.json) | JSON | -36 | 0 | 0 | -36 |
| [vendordeps/Phoenix5.json](/vendordeps/Phoenix5.json) | JSON | -151 | 0 | 0 | -151 |
| [vendordeps/Phoenix6.json](/vendordeps/Phoenix6.json) | JSON | -339 | 0 | 0 | -339 |
| [vendordeps/REVLib.json](/vendordeps/REVLib.json) | JSON | -74 | 0 | 0 | -74 |
| [vendordeps/WPILibNewCommands.json](/vendordeps/WPILibNewCommands.json) | JSON | -38 | 0 | -1 | -39 |
| [vendordeps/photonlib.json](/vendordeps/photonlib.json) | JSON | -57 | 0 | 0 | -57 |

[Summary](results.md) / [Details](details.md) / [Diff Summary](diff.md) / Diff Details