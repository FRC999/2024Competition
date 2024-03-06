package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class TrajectoryHelpers {

    /**
     * This method will adjust the endPose based on the angle.
     * For now it will siply turn it to the specified angle using the startPose as origin.
     * This will likely be sufficient for small trajectory deviations during auto-driving to pickup notes
     */
    public static Pose2d correctEndingPoseBasedOnNoteLocation(Pose2d startPose, Pose2d endPose, double angle) {
        Pose2d relativePoseTurned = endPose.relativeTo(startPose).rotateBy(Rotation2d.fromDegrees(angle));
        Transform2d t = relativePoseTurned.minus(new Pose2d()); // converts relative pose to a transformation
        return startPose.transformBy(t);
        
    }

}
