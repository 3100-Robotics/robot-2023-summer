package frc.robot.vision;

import org.photonvision.targeting.PhotonPipelineResult;
import frc.robot.Constants.visionConstants.cameraType;
import org.photonvision.targeting.PhotonTrackedTarget;

public class results {

    PhotonPipelineResult photonResult;
    cameraType type;

    /**
     * a simple class to allow having one variable that can be either a photon
     * result or a limelight result (its here so if I do reimplement limelight
     * support I can easily do so)
     * @param results the photon results
     * @param type the type of the camera
     */
    public results(PhotonPipelineResult results, cameraType type) {
        photonResult = results;
        this.type = type;
    }

    /**
     * @return whether there are targets
     */
    public boolean hasTargets() {
        return photonResult.hasTargets();
    }

    /**
     * @return the best target in view. photon only
     */
    public PhotonTrackedTarget getBestTarget() {
        return photonResult.getBestTarget();
    }
}
