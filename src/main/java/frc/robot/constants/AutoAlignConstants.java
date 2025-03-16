package frc.robot.constants;

import java.util.HashMap;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class AutoAlignConstants {
    public static final PathConstraints PATH_PLANNER_CONSTRAINTS = new PathConstraints(
            2.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540));

    public static final double FORWARDS_SPEED = 0.5; // speed until stall
    public static final double VEL_THRESHOLD = 0.1; // velocity of stall threshold
    public static final double CURRENT_THRESHOLD = 50; // current of stall threshold

    private static final Rotation2d AB = new Rotation2d(Math.toRadians(0));
    private static final Rotation2d CD = new Rotation2d(Math.toRadians(60));
    private static final Rotation2d EF = new Rotation2d(Math.toRadians(120));
    private static final Rotation2d GH = new Rotation2d(Math.toRadians(180));
    private static final Rotation2d IJ = new Rotation2d(Math.toRadians(240));
    private static final Rotation2d KL = new Rotation2d(Math.toRadians(300));

    public static final Pose2d REEF_A = new Pose2d(3.213, 4.204, AB);
    public static final Pose2d REEF_B = new Pose2d(3.245, 3.821, AB);
    public static final Pose2d REEF_C = new Pose2d(3.65, 2.97, CD);
    public static final Pose2d REEF_D = new Pose2d(4.04, 2.76, CD);
    public static final Pose2d REEF_E = new Pose2d(5.39, 2.51, EF);
    public static final Pose2d REEF_F = new Pose2d(5.77, 2.70, EF);
    public static final Pose2d REEF_G = new Pose2d(5.96, 3.83, GH);
    public static final Pose2d REEF_H = new Pose2d(5.96, 4.22, GH);
    public static final Pose2d REEF_I = new Pose2d(5.274, 5.037, IJ);
    public static final Pose2d REEF_J = new Pose2d(4.939, 5.198, IJ);
    public static final Pose2d REEF_K = new Pose2d(3.989, 5.201, KL);
    public static final Pose2d REEF_L = new Pose2d(3.707, 5.010, KL);

    public static final Pose2d ALGAE_AB = new Pose2d(3.1876, 4.026, AB);
    public static final Pose2d ALGAE_CD = new Pose2d(3.839, 2.899, CD);
    public static final Pose2d ALGAE_EF = new Pose2d(5.1396, 2.899, EF);
    public static final Pose2d ALGAE_GH = new Pose2d(5.7909, 4.0258, GH);
    public static final Pose2d ALGAE_IJ = new Pose2d(5.1396, 5.15242, IJ);
    public static final Pose2d ALGAE_KL = new Pose2d(3.8389, 5.1524, KL);

    public static final HashMap<AutoAlignPosition, Pose2d> REEF_POSITIONS = new HashMap<AutoAlignPosition, Pose2d>() {
        {
            put(AutoAlignPosition.A, REEF_A);
            put(AutoAlignPosition.B, REEF_B);
            put(AutoAlignPosition.C, REEF_C);
            put(AutoAlignPosition.D, REEF_D);
            put(AutoAlignPosition.E, REEF_E);
            put(AutoAlignPosition.F, REEF_F);
            put(AutoAlignPosition.G, REEF_G);
            put(AutoAlignPosition.H, REEF_H);
            put(AutoAlignPosition.I, REEF_I);
            put(AutoAlignPosition.J, REEF_J);
            put(AutoAlignPosition.K, REEF_K);
            put(AutoAlignPosition.L, REEF_L);
        }
    };

    public static final HashMap<AutoAlignPosition, Pose2d> ALGAE_POSITIONS = new HashMap<AutoAlignPosition, Pose2d>() {
        {
            put(AutoAlignPosition.A, ALGAE_AB);
            put(AutoAlignPosition.B, ALGAE_AB);
            put(AutoAlignPosition.C, ALGAE_CD);
            put(AutoAlignPosition.D, ALGAE_CD);
            put(AutoAlignPosition.E, ALGAE_EF);
            put(AutoAlignPosition.F, ALGAE_EF);
            put(AutoAlignPosition.G, ALGAE_GH);
            put(AutoAlignPosition.H, ALGAE_GH);
            put(AutoAlignPosition.I, ALGAE_IJ);
            put(AutoAlignPosition.J, ALGAE_IJ);
            put(AutoAlignPosition.K, ALGAE_KL);
            put(AutoAlignPosition.L, ALGAE_KL);
        }
    };
}
