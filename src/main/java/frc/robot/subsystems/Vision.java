package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.visionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    private final PhotonCamera frontCamera = new PhotonCamera("frontCamera");
    private final PhotonCamera backCamera = new PhotonCamera("backCamera");

    public Vision() {}

    public PhotonPipelineResult getFrontCameraResults() {
        return frontCamera.getLatestResult();
    }

    public PhotonPipelineResult getBackCameraResults() {
        return backCamera.getLatestResult();
    }

    public double calculateAngle(String level) {
        PhotonPipelineResult frontResults = getFrontCameraResults();
        PhotonPipelineResult backResults = getBackCameraResults();

        PhotonTrackedTarget frontBestTarget = null;
        PhotonTrackedTarget backBestTarget = null;

        double results = 0;

        double distance = 0;

        int numLevel;

        if (level.equals("mid")) {
            numLevel = 0;
        }
        else {
            numLevel = 1;
        }

        if (frontResults.hasTargets()) {
                frontBestTarget = frontResults.getBestTarget();
                distance = frontBestTarget.getBestCameraToTarget().getX();
        }
        else if (backResults.hasTargets()) {
            backBestTarget = backResults.getBestTarget();
            distance = backBestTarget.getBestCameraToTarget().getX();
        }

        results = Math.atan(
                (2/distance) *
                        (visionConstants.heightDiffs[numLevel] + visionConstants.maxHeight +
                                Math.sqrt(Math.pow(visionConstants.maxHeight, 2) +
                                        visionConstants.heightDiffs[numLevel] * visionConstants.maxHeight)));

        return results;
    }

    public double calculateSpeed(String level) {
        int numLevel;
        if (level.equals("mid")) {
            numLevel = 0;
        }
        else {
            numLevel = 1;
        }
        return (Math.sqrt(2*visionConstants.g*
                (visionConstants.heightDiffs[numLevel] + visionConstants.maxHeight)))/Math.sin(calculateAngle(level));
    }
}

