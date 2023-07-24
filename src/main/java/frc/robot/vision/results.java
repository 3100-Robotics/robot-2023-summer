package frc.robot.vision;

import frc.robot.LimelightHelpers;
import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.Constants.visionConstants.cameraType;
import org.photonvision.targeting.PhotonTrackedTarget;

public class results {

    PhotonPipelineResult photonResult;
    LimelightHelpers.LimelightResults LLResult;
    cameraType type;

    /**
     * a simple class to allow having one variable that can be either a photon result or a limelight result
     * @param results the photon results
     * @param type the type of the camera
     */
    public results(PhotonPipelineResult results, cameraType type) {
        photonResult = results;
        this.type = type;
    }

    /**
     * a simple class to allow having one variable that can be either a photon result or a limelight result
     * @param results the limelight results
     * @param type the type of the camera
     */
    public results(LimelightHelpers.LimelightResults results, cameraType type) {
        LLResult = results;
        this.type = type;
    }

    /**
     * @return whether there are targets
     */
    public boolean hasTargets() {
        if (type.equals(cameraType.photonVision)) {
            return photonResult.hasTargets();
        }
        return LLResult.targetingResults.valid;
    }

    /**
     * @return the best target in view. photon only
     */
    public PhotonTrackedTarget getBestTarget() {
        return photonResult.getBestTarget();
    }
}
