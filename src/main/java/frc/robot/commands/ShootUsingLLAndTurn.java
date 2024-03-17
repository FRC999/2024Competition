// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants.autoPoses;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.TrajectoryHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootUsingLLAndTurn extends SequentialCommandGroup {
  private Pose2d originPose = new Pose2d(0,0,Rotation2d.fromDegrees(0));
  /** Creates a new ShootUsingLL. */
  public ShootUsingLLAndTurn() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(  

      new ConditionalCommand(

      // ============================= ON TRUE ================================================
      new DeferredCommand(
          () -> 
          new PrintCommand("Shooting Distance : " + (RobotContainer.llVisionSubsystem.getShootingDistance() - 1.1)), 
           Set.of()).andThen(
        
       // Turn to the AT
        
        new DeferredCommand(
          // ========= ROTATE TO THE NEW POSE USING TURN TO REL ANGLE ========
        /*   () -> new TurnToRelativeAngleSoftwarePIDCommand(
            () -> new Pose2d(
                0,  // x
                0,  // y
                TrajectoryHelpers.rotateToPointToSecondPose(
                  RobotContainer.llVisionSubsystem.getRobotFieldPoseLL().plus
                    (new Transform2d
                      (new Translation2d(), 
                      Rotation2d.fromDegrees(180)
                      )
                    ) ,
                    autoPoses.BLUE_SPEAKER_TAG.getPose()
                )
              )  //rotation
              .getRotation()  // as Rotation2d
            )
            */
            () -> new PrintCommand( "A:"+
               TrajectoryHelpers.rotateToPointToSecondPose(
                  RobotContainer.llVisionSubsystem.getRobotFieldPoseLL().plus
                    (new Transform2d
                      (new Translation2d(), 
                      Rotation2d.fromDegrees(180)
                      )
                    ) ,
                    autoPoses.BLUE_SPEAKER_TAG.getPose()
               
                
                //rotation
             
                )
            )
            , 
            Set.of()
            )
        )
        .andThen(

        new PrintCommand(
          RobotContainer.llVisionSubsystem.getRobotFieldPoseLL().toString()
        
          
          ) 
        )
        .andThen(

         // new ShootUsingLL()
          
        ) ,

       // FOR TESTING THE CORRECT ROTATIONS AND POSES
         /*  new DeferredCommand(
            () -> new PrintCommand("FP: " + autoPoses.BLUE_SPEAKER_TAG.getPose() + 
            " \n" + "SP:"+ 
              new Pose2d(0,0,
                TrajectoryHelpers.rotateToPointToSecondPose(
                  RobotContainer.llVisionSubsystem.getRobotFieldPoseLL().plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))),
                  autoPoses.BLUE_SPEAKER_TAG.getPose()
                  )
              )+" \n"+"RP:"+RobotContainer.llVisionSubsystem.getRobotFieldPoseLL().plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)))
            ), 
            Set.of()
          ) */
          
          
        // ====================== ON FALSE ======================================================
        new PrintCommand("No AT Visible"),

        // ========================= CONDITIONAL ==============================================

        () -> RobotContainer.llVisionSubsystem.isApriltagVisible() && LimelightHelpers.isInRange(RobotContainer.llVisionSubsystem.getShootingDistance(), 0.0, 4.0)
      
      )
    
    );
      
  }
}
