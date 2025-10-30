package frc.robot.subsystems.vision;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final Camera frontCam;
    private final Camera backCam;
    private final VisionSystemSim visionSim;
    private final EagleSwerveDrivetrain drivetrain;

    public Vision(EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        frontCam = new Camera(
                VisionConstants.FRONT_CAMERA_NAME,
                VisionConstants.FRONT_CAMERA_POSITION,
                (PoseObservation observation) -> drivetrain.addVisionMeasurement(
                        observation.pose(),
                        observation.timestamp(),
                        observation.standardDeviations()),
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds);
        
        backCam = new Camera(
            VisionConstants.BACK_CAMERA_NAME,
            VisionConstants.BACK_CAMERA_POSITION,
            (PoseObservation observation) -> drivetrain.addVisionMeasurement(
                    observation.pose(),
                    observation.timestamp(),
                    observation.standardDeviations()),
            () -> drivetrain.getState().Pose,
            () -> drivetrain.getState().Speeds);

        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.aprilTagFieldLayout);
            visionSim.addCamera(frontCam.getSimCamera(), VisionConstants.FRONT_CAMERA_POSITION);
            visionSim.addCamera(backCam.getSimCamera(), VisionConstants.BACK_CAMERA_POSITION);
        } else {
            visionSim = null;
        }
    }

    @Override
    public void periodic() {
        frontCam.periodic();
        backCam.periodic();
        if (RobotBase.isSimulation()) {
            visionSim.update(drivetrain.getState().Pose);
        }
    }
}
