package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

public class AprilTagHelper {
    public final PhotonCamera cam = new PhotonCamera("ov9281");
    // distance from robot to camera
    Transform3d robotToCam = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5), // camera placement on robot
            new Rotation3d(0, 0, 0));

    AprilTagFieldLayout aprilTagFieldLayout;
    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    static PhotonPoseEstimator poseEstimator;

    public AprilTagHelper() {
        init();
    }

    public List<Pose3d> aprilTagPoses()
	{
		// PhotonPipelineResult result = AprilTagHelper.cam.getLatestResult();
		// PhotonTrackedTarget target = result.getBestTarget();
		// List<PhotonTrackedTarget> targets = result.getTargets();

		ArrayList<Pose3d> aprilTagPoses = new ArrayList<>();
		for (int i = 1; i < 9; i++) {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(i);
            if(tagPose.isPresent())
            {
                aprilTagPoses.add(tagPose.get());
            }
		}

        return aprilTagPoses;
	}

    

    private void init() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, cam,
                robotToCam);
    }

    /**
     * 
     * @param prevEstimatedRobotPose
     * @return an EstimatedRobotPose which includes a Pose3d of the latest estimated
     *         pose (using the selected strategy) along with a double of the 
     *         timestamp when the robot pose was estimated
     * Use this to update drivetrain pose estimator
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }

    // unfiltered view of camera
    public void toggleDriverMode() {
        if (cam.getDriverMode()) {
            cam.setDriverMode(false);
        }

        else {
            cam.setDriverMode(true);
        }
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        var result = cam.getLatestResult();
        return result != null ? Optional.of(result) : Optional.empty();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        var latestOpt = getLatestResult();
        if (latestOpt.isPresent()) {
            if (latestOpt.get().hasTargets()) {
                return Optional.of(latestOpt.get().getBestTarget());
            }
        }

        return Optional.empty();
    }
}
