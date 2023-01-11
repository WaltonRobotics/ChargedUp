package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagHelper {
    public static final PhotonCamera cam = new PhotonCamera("camera");  

    public static PhotonPipelineResult getLatestResult(){
        return cam.getLatestResult();
    }

    public static PhotonPipelineResult getBestResult(){
        return cam.getBestResult();
    }
}
