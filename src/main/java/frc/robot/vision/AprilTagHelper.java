package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class AprilTagHelper {
    public static final PhotonCamera cam = new PhotonCamera("ov9281");
    // distance from robot to camera
    Transform3d robotToCam = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));

    AprilTagFieldLayout aprilTagFieldLayout;
    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    static PhotonPoseEstimator poseEstimator;

    public AprilTagHelper() {
        init();
    }

    private void init() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, cam,
                robotToCam);
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose2d(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }

    // unfiltered view of camera
    public static void toggleDriverMode() {
        if (cam.getDriverMode()) {
            cam.setDriverMode(false);
        }

        else {
            cam.setDriverMode(true);
        }
    }

    public static Optional<PhotonPipelineResult> getLatestResult() {
        var result = cam.getLatestResult();
        return result != null ? Optional.of(result) : Optional.empty();
    }

    public static Optional<PhotonTrackedTarget> getBestTarget() {
        var latestOpt = getLatestResult();
        if (latestOpt.isPresent()) {
            if (latestOpt.get().hasTargets()) {
                return Optional.of(latestOpt.get().getBestTarget());
            }
        }

        return Optional.empty();
    }
}
