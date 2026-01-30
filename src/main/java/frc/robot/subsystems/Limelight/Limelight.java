package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;

public class Limelight {
    private static final String limelightName = "limelight";

    public Limelight() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    public Pose2d getPose() {
        return LimelightHelpers.getBotPose2d(limelightName);
    }
}
