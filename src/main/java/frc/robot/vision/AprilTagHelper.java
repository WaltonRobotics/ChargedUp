package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagHelper {
    public static final PhotonCamera cam = new PhotonCamera("camera");
    //distance from robot to camera
    Transform3d robotToCam = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5), 
        new Rotation3d(0,0,0)
        );

        AprilTagFieldLayout aprilTagFieldLayout;
        ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        RobotPoseEstimator robotPoseEstimator; 

    public AprilTagHelper(){
        init();
        
    }
    
    private void init(){
        try {
            aprilTagFieldLayout =  AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            //TODO: write warning statement
        } 

        camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));
        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    public static PhotonPipelineResult getLatestResult(){
        return cam.getLatestResult();
    }

    public static PhotonTrackedTarget  getBestTarget() {
        PhotonPipelineResult result = getLatestResult();
        return result.getBestTarget();
    }

    public static double getYaw() {
        PhotonTrackedTarget target = getBestTarget();
        return target.getYaw();
    }
    
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    
        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    //unfiltered view of camera 
    public void toggleDriverMode(){
        if(cam.getDriverMode()){
        cam.setDriverMode(false);
        }

        else{
            cam.setDriverMode(true);
        }
        
    }
}
