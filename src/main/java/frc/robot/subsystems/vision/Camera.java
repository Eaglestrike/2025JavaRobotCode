// adapted from https://github.com/gwhs/2025-Reefscape/blob/dev/src/main/java/frc/robot/subsystems/aprilTagCam/AprilTagCam.java#L164-L242
package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable visionTable = inst.getTable("Vision");
    private final NetworkTable cameraTable;

    private final StructPublisher<Pose3d> cameraPoseTopic;
    private final StructPublisher<Pose2d> acceptedPoseTopic;
    private final DoublePublisher acceptedTimestampTopic;
    private final DoubleArrayPublisher acceptedStdevTopic;
    private final BooleanPublisher isConnectedTopic;
    private final StructPublisher<Pose3d> rejectedPoseTopic;
    private final StringPublisher rejectedReasonTopic;
    private final StructArrayPublisher<Pose3d> tagsSeenTopic;

    private final PhotonCamera cam;
    private final PhotonCameraSim cameraSim;

    private final Consumer<PoseObservation> addVisionMeasurement;
    private final PhotonPoseEstimator photonEstimator;
    private final Transform3d robotToCam;
    private final Supplier<Pose2d> currRobotPose;
    private final Supplier<ChassisSpeeds> currRobotSpeed;
    private boolean isConnected;

    private final Alert visionNotConnected;

    Optional<EstimatedRobotPose> optionalEstimPose;

    public Camera(
            String cameraName,
            Transform3d robotToCam,
            Consumer<PoseObservation> addVisionMeasurement,
            Supplier<Pose2d> currRobotPose,
            Supplier<ChassisSpeeds> currRobotSpeed) {
        cam = new PhotonCamera(cameraName);
        this.addVisionMeasurement = addVisionMeasurement;
        this.robotToCam = robotToCam;
        this.currRobotPose = currRobotPose;
        this.currRobotSpeed = currRobotSpeed;

        visionNotConnected = new Alert(cameraName + " NOT CONNECTED", AlertType.kError);

        photonEstimator = new PhotonPoseEstimator(
                VisionConstants.aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam);

        cameraTable = visionTable.getSubTable(cameraName);
        cameraPoseTopic = cameraTable.getStructTopic("Camera Pose", Pose3d.struct)
                .publish();
        acceptedPoseTopic = cameraTable.getStructTopic("Accepted Pose", Pose2d.struct)
                .publish();
        acceptedTimestampTopic = cameraTable.getDoubleTopic("Accepted Timestamp").publish();
        acceptedStdevTopic = cameraTable.getDoubleArrayTopic("Accepted Stdev")
                .publish();
        isConnectedTopic = cameraTable.getBooleanTopic("Is Connected").publish();
        rejectedPoseTopic = cameraTable.getStructTopic("Rejected Pose", Pose3d.struct)
                .publish();
        rejectedReasonTopic = cameraTable.getStringTopic("Rejected Reason").publish();
        tagsSeenTopic = cameraTable
                .getStructArrayTopic("April Tags Seen", Pose3d.struct).publish();

        if (Robot.isSimulation()) {
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            cameraSim = new PhotonCameraSim(cam, cameraProp);
            cameraSim.enableDrawWireframe(true);
        } else {
            cameraSim = null;
        }
    }

    public void periodic() {
        isConnected = cam.isConnected();
        Pose2d robotPose = currRobotPose.get();
        Pose3d robotPose3d = new Pose3d(robotPose);
        Pose3d cameraPose3d = robotPose3d.plus(robotToCam);
        cameraPoseTopic.set(cameraPose3d);

        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        if (results.isEmpty()) {
            return;
        }

        photonEstimator.addHeadingData(Utils.getCurrentTimeSeconds(), robotPose.getRotation());

        for (PhotonPipelineResult targetPose : results) {
            boolean hasInvalidTag = false;
            for (var target : targetPose.getTargets()){
                for(int ignoredTag : VisionConstants.IGNORE_TAGS){
                    if(target.getFiducialId() == ignoredTag){
                        hasInvalidTag = true;
                        break;
                    }
                }
            }

            if(hasInvalidTag){
                continue;
            }

            optionalEstimPose = photonEstimator.update(targetPose,
                    cam.getCameraMatrix(),
                    cam.getDistCoeffs(),
                    Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(true, 0.0)));

            if (optionalEstimPose.isEmpty()) {
                continue;
            }

            Pose3d estimPose3d = optionalEstimPose.get().estimatedPose;

            if (!filterResults(estimPose3d, optionalEstimPose.get(), currRobotSpeed.get())) {
                continue;
            }

            Pose2d pos = estimPose3d.toPose2d();
            // double timestamp = Utils.fpgaToCurrentTime(targetPose.getTimestampSeconds());
            double timestamp = targetPose.getTimestampSeconds();
            Matrix<N3, N1> standardDeviations = computeStandardDeviations(optionalEstimPose,
                    optionalEstimPose.get().targetsUsed);

            acceptedPoseTopic.set(pos);
            acceptedTimestampTopic.set(timestamp);
            acceptedStdevTopic.set(getSDArray(standardDeviations));

            addVisionMeasurement.accept(new PoseObservation(
                    timestamp,
                    pos,
                    standardDeviations));
        }

        isConnectedTopic.set(isConnected);
        visionNotConnected.set(!isConnected);
    }

    public static double[] getSDArray(Matrix<N3, N1> sd) {
        double[] sdArray = new double[3];
        for (int i = 0; i < 3; i++) {
            sdArray[i] = sd.get(i, 0);
        }
        return sdArray;
    }

    public boolean filterResults(
            Pose3d estimPose3d, EstimatedRobotPose optionalEstimPose, ChassisSpeeds speed) {
        if (estimPose3d.getZ() > VisionConstants.Z_TOLERANCE || estimPose3d.getZ() < -VisionConstants.Z_TOLERANCE) {
            // change if we find out that z starts from camera height
            rejectedPoseTopic.set(estimPose3d);
            rejectedReasonTopic.set("out of Z bounds, Z: " + estimPose3d.getZ());
            return false;
        }

        // outside of field
        double upperXBound = VisionConstants.MAX_X_VALUE + VisionConstants.XY_TOLERANCE;
        double upperYBound = VisionConstants.MAX_Y_VALUE + VisionConstants.XY_TOLERANCE;
        double lowerXYBound = -VisionConstants.XY_TOLERANCE;
        if (estimPose3d.getX() < lowerXYBound || estimPose3d.getY() < lowerXYBound) {
            rejectedPoseTopic.set(estimPose3d);
            rejectedReasonTopic.set(
                    "Y or X is less than 0, X: " + estimPose3d.getX() + ", Y: " + estimPose3d.getY());
            return false;
        }
        if (estimPose3d.getX() > upperXBound || estimPose3d.getY() > upperYBound) {
            rejectedPoseTopic.set(estimPose3d);
            rejectedReasonTopic.set(
                    "Y or X is out of bounds, X: " + estimPose3d.getX() + ", Y: " + estimPose3d.getY());
            return false;
        }

        // tags too far away
        double averageDistance = 0;
        double numOfTags = 0;
        ArrayList<Pose3d> tagList = new ArrayList<Pose3d>();
        for (PhotonTrackedTarget target : optionalEstimPose.targetsUsed) {
            Optional<Pose3d> tagPoseOptional = VisionConstants.aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPoseOptional.isEmpty()) {
                continue;
            }
            Pose3d tagPose = tagPoseOptional.get();
            double distance = optionalEstimPose.estimatedPose.minus(tagPose).getTranslation().getNorm();
            averageDistance += distance;
            numOfTags++;
            tagList.add(tagPose);
        }

        tagsSeenTopic.set(tagList.toArray(new Pose3d[0]));
        if (numOfTags > 0) {
            averageDistance /= numOfTags;
        }
        if (numOfTags == 1 && averageDistance > VisionConstants.SINGLE_APRILTAG_MAX_DISTANCE) {
            rejectedPoseTopic.set(estimPose3d);
            rejectedReasonTopic.set("Too far of distance to april tag, distance: " +
                    averageDistance);
            return false;
        } else if (numOfTags > 1
                && averageDistance > VisionConstants.MULTI_APRILTAG_MAX_DISTANCE) {
            rejectedPoseTopic.set(estimPose3d);
            rejectedReasonTopic.set("Too far of distance to april tag, distance: " +
                    averageDistance);
            return false;
        }

        // drivebase velocity or rotaion is too high
        double xVel = speed.vxMetersPerSecond;
        double yVel = speed.vyMetersPerSecond;
        double vel = Math.sqrt(Math.pow(yVel, 2) + Math.pow(xVel, 2));
        double rotation = speed.omegaRadiansPerSecond;

        if (vel > VisionConstants.MAX_VELOCITY || rotation > VisionConstants.MAX_ROTATION) {
            rejectedPoseTopic.set(estimPose3d);
            rejectedReasonTopic.set(
                    "Velocity/Rotation is too fast, Velocity: " + vel + ", Rotation: " + rotation);
            return false;
        }

        return true;
    }

    private Matrix<N3, N1> computeStandardDeviations(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            return VisionConstants.kSingleTagStdDevs;
        } else {
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                return VisionConstants.kSingleTagStdDevs;
            } else {
                avgDist /= numTags;
                if (numTags > 1)
                    estStdDevs = VisionConstants.kMultiTagStdDevs;
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                return estStdDevs;
            }
        }
    }

    public PhotonCameraSim getSimCamera() {
        return cameraSim;
    }
}