package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout;
    static {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
    }

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.1, 0.1, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 1);

    public static final ArrayList<Integer> IGNORE_TAGS = new ArrayList<>(List.of(13, 14, 12, 16, 15, 5, 4, 3, 2, 1));

    public static final String FRONT_CAMERA_NAME = "CAMERA_FRONT";
    public static final Transform3d FRONT_CAMERA_POSITION = new Transform3d(
            Units.inchesToMeters(9.524),
            Units.inchesToMeters(0),
            Units.inchesToMeters(8.250),
            new Rotation3d(
                    Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));

    public static final String BACK_CAMERA_NAME = "CAMERA_BACK";
    public static final Transform3d BACK_CAMERA_POSITION = new Transform3d(
            Units.inchesToMeters(-9.524),
            Units.inchesToMeters(0),
            Units.inchesToMeters(8.250),
            new Rotation3d(
                    Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)));

    public static final double Z_TOLERANCE = 2.00;
    public static final double XY_TOLERANCE = 2.00;
    public static final double MAX_X_VALUE = 690.87;
    public static final double MAX_Y_VALUE = 317.00;
    public static final double SINGLE_APRILTAG_MAX_DISTANCE = 3;
    public static final double MULTI_APRILTAG_MAX_DISTANCE = 4.3;
    public static final double MAX_VELOCITY = 4;
    public static final double MAX_ROTATION = Math.PI;

}
