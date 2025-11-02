package frc.robot.subsystems.endeffector;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.AutoAlignPosition;
import frc.robot.constants.ReefPositions;

public class EndEffectorSideUtils {

    public static final List<AutoAlignPosition> REEF_POSITIONS = List.of(
            AutoAlignPosition.ALGAE_AB,
            AutoAlignPosition.ALGAE_CD,
            AutoAlignPosition.ALGAE_EF,
            AutoAlignPosition.ALGAE_GH,
            AutoAlignPosition.ALGAE_IJ,
            AutoAlignPosition.ALGAE_KL);

    public static List<Pose2d> getCenterPoses() {
        ArrayList<Pose2d> centerPoses = new ArrayList<>();
        for (AutoAlignPosition position : REEF_POSITIONS) {
            centerPoses.add(ReefPositions.getReefPosition(DriverStation.getAlliance().orElse(Alliance.Blue), position));
        }
        return centerPoses;
    }

    public static boolean facingReef(Pose2d currentPose) {
        Pose2d closestReefPose = currentPose.nearest(getCenterPoses());
        double angleError = Math.abs(currentPose.getRotation().minus(closestReefPose.getRotation()).getRadians());

        // System.out.println("Angle Error: " + angleError);

        if (angleError < Math.PI / 2.0) {
            return true;
        }
        return false;
    }
}
