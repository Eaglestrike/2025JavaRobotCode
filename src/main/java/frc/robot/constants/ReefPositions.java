package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefPositions {
    public static final Distance REEF_CENTER_TO_BRANCH = Inches.of(6.633);
    // public static final Distance REEF_CENTER_TO_BRANCH = Meters.of(4.01 - 3.54); // FOUND EMPERICALLY, ONLY WORKS FOR LEFT
    // public static final Distance BEHIND_REEF_CENTER = Inches.of(16);
    public static final Distance BEHIND_REEF_CENTER = Inches.of(18);

    private static final NetworkTable autoAlignTable = NetworkTableInstance.getDefault().getTable("Reef Debugging");
    private static final StructPublisher<Pose2d> tagPosePublisher = autoAlignTable.getStructTopic("Tag Pose", Pose2d.struct).publish();
    private static final StructPublisher<Pose2d> alignOffsetPublisher = autoAlignTable.getStructTopic("No rotation Pose", Pose2d.struct).publish();
    private static final StructPublisher<Pose2d> alignPosePublisher = autoAlignTable.getStructTopic("Align Pose", Pose2d.struct).publish();

    public static Pose2d getReefAlignPose(int tagId, boolean invertOffset) {
        Pose2d tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(tagId).get().toPose2d();
        tagPosePublisher.set(tagPose);
        Translation2d alignOffsetRel = new Translation2d(
                -BEHIND_REEF_CENTER.in(Meters),
                ((invertOffset ? -1 : 1) * REEF_CENTER_TO_BRANCH.in(Meters)));

        Translation2d alignOffset = alignOffsetRel.rotateBy(tagPose.getRotation().minus(new Rotation2d(Math.PI)));

        Translation2d alignTrans = tagPose.getTranslation().plus(alignOffset);
        Pose2d alignPose = new Pose2d(alignTrans, tagPose.getRotation().plus(new Rotation2d(Math.PI)));
        alignPosePublisher.set(alignPose);
        return alignPose;
    }

    public static Pose2d getAlgaeAlignPose(int tagId) {
        Pose2d tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(tagId).get().toPose2d();
        Translation2d alignOffsetRel = new Translation2d(
                -BEHIND_REEF_CENTER.in(Meters), 0);
        Translation2d alignOffset = alignOffsetRel.rotateBy(tagPose.getRotation().minus(new Rotation2d(Math.PI)));
        Translation2d alignTrans = tagPose.getTranslation().plus(alignOffset);
        Pose2d alignPose = new Pose2d(alignTrans, tagPose.getRotation().plus(new Rotation2d(Math.PI)));
        return alignPose;
    }

    static Map<AutoAlignPosition, Pose2d> reefPositionsBlue = Map.ofEntries(
            Map.entry(AutoAlignPosition.A, getReefAlignPose(18, false)),
            Map.entry(AutoAlignPosition.B, getReefAlignPose(18, true)),
            Map.entry(AutoAlignPosition.C, getReefAlignPose(17, false)),
            Map.entry(AutoAlignPosition.D, getReefAlignPose(17, true)),
            Map.entry(AutoAlignPosition.E, getReefAlignPose(22, false)),
            Map.entry(AutoAlignPosition.F, getReefAlignPose(22, true)),
            Map.entry(AutoAlignPosition.G, getReefAlignPose(21, false)),
            Map.entry(AutoAlignPosition.H, getReefAlignPose(21, true)),
            Map.entry(AutoAlignPosition.I, getReefAlignPose(20, false)),
            Map.entry(AutoAlignPosition.J, getReefAlignPose(20, true)),
            Map.entry(AutoAlignPosition.K, getReefAlignPose(19, false)),
            Map.entry(AutoAlignPosition.L, getReefAlignPose(19, true)),
            Map.entry(AutoAlignPosition.ALGAE_AB, getAlgaeAlignPose(18)),
            Map.entry(AutoAlignPosition.ALGAE_CD, getAlgaeAlignPose(17)),
            Map.entry(AutoAlignPosition.ALGAE_EF, getAlgaeAlignPose(22)),
            Map.entry(AutoAlignPosition.ALGAE_GH, getAlgaeAlignPose(21)),
            Map.entry(AutoAlignPosition.ALGAE_IJ, getAlgaeAlignPose(20)),
            Map.entry(AutoAlignPosition.ALGAE_KL, getAlgaeAlignPose(19)));

    static Map<AutoAlignPosition, Pose2d> reefPositionsRed = Map.ofEntries(
            Map.entry(AutoAlignPosition.A, getReefAlignPose(7, false)),
            Map.entry(AutoAlignPosition.B, getReefAlignPose(7, true)),
            Map.entry(AutoAlignPosition.C, getReefAlignPose(8, false)),
            Map.entry(AutoAlignPosition.D, getReefAlignPose(8, true)),
            Map.entry(AutoAlignPosition.E, getReefAlignPose(9, false)),
            Map.entry(AutoAlignPosition.F, getReefAlignPose(9, true)),
            Map.entry(AutoAlignPosition.G, getReefAlignPose(10, false)),
            Map.entry(AutoAlignPosition.H, getReefAlignPose(10, true)),
            Map.entry(AutoAlignPosition.I, getReefAlignPose(11, false)),
            Map.entry(AutoAlignPosition.J, getReefAlignPose(11, true)),
            Map.entry(AutoAlignPosition.K, getReefAlignPose(6, false)),
            Map.entry(AutoAlignPosition.L, getReefAlignPose(6, true)),
            Map.entry(AutoAlignPosition.ALGAE_AB, getAlgaeAlignPose(7)),
            Map.entry(AutoAlignPosition.ALGAE_CD, getAlgaeAlignPose(8)),
            Map.entry(AutoAlignPosition.ALGAE_EF, getAlgaeAlignPose(9)),
            Map.entry(AutoAlignPosition.ALGAE_GH, getAlgaeAlignPose(10)),
            Map.entry(AutoAlignPosition.ALGAE_IJ, getAlgaeAlignPose(11)),
            Map.entry(AutoAlignPosition.ALGAE_KL, getAlgaeAlignPose(6)));

    public static Pose2d getReefPosition(Alliance alliance, AutoAlignPosition position) {
        return alliance == Alliance.Blue ? reefPositionsBlue.get(position) : reefPositionsRed.get(position);
    }
}
