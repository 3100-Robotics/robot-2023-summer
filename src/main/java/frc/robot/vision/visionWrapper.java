package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import frc.robot.Constants.visionConstants.cameraType;

import java.io.IOException;
import java.util.*;


/**
 * This class creates a light wrapper for the {@link PhotonCamera} and
 * {@link PhotonPoseEstimator}.
 */
public class visionWrapper {

    private PhotonCamera photonCamera;

    private PhotonPoseEstimator poseEstimator;

    private final cameraType type;

    /**
     * constructs a new vision wrapper object using the name of the photonCamera
     * and the position on the robot
     * @param cameraName the name of the photonCamera
     * @param robotToCam a {@link Transform3d} from the center of the robot to the
     *                   position of the photonCamera
     */
    public visionWrapper(String cameraName, Transform3d robotToCam, cameraType type) {

        this.type = type;

        if (type.equals(cameraType.photonVision)) {
            // set up a photon vision camera
            photonCamera = new PhotonCamera(cameraName);

            // if it is a simulation, disable some error throwing
            if (RobotBase.isSimulation()) {
                PhotonCamera.setVersionCheckEnabled(false);
            }

            try {
                // Attempt to load the AprilTagFieldLayout that will tell us where
                // the tags are on the field.
                AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
                // Create pose estimator
                poseEstimator =
                        new PhotonPoseEstimator(
                                fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, photonCamera, robotToCam);
                poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            } catch (IOException e) {
                // The AprilTagFieldLayout failed to load. We won't be able to
                // estimate poses if we don't know where the tags are.
                DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                poseEstimator = null;
            }
        }
    }

    /**
     * get the latest results from the camera
     * @return the latest results
     */
    public results getLatestResult() {
        return new results(photonCamera.getLatestResult(), type);
    }

    /**
     * A function to estimate the global pose of the robot according to the april
     * tags in view of the robot
     * @param prevEstimatedRobotPose the previous robot pose
     * @return the estimated pose of the robot according to this photonCamera
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (poseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }

        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }
}
