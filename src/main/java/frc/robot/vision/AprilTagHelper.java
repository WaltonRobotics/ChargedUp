package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagHelper {
    public static final PhotonCamera cam = new PhotonCamera("camera");  

    public static PhotonPipelineResult getLatestResult(){
        return cam.getLatestResult();
    }

    public static PhotonTrackedTarget  getBestTarget() {
        PhotonPipelineResult result = getLatestResult();
        return result.getBestTarget();
    }
}
