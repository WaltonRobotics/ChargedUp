package frc.robot.vision;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import frc.lib.vision.EstimatedRobotPose;
// import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import frc.lib.vision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator;
import frc.lib.vision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class AprilTagCamera {
    public final PhotonCamera rightLowCam = new PhotonCamera("LeftHighCam");
    public final PhotonCamera leftLowCam = new PhotonCamera("LeftLowCam"); // TODO: name the camera (will do when we have the actual camera)
    // distance from robot to camera
    private final Transform3d robotToCam1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(9.279), Units.inchesToMeters(-9.52), Units.inchesToMeters(9.125)), // camera placement on robot
            new Rotation3d(0, Units.degreesToRadians(14), Units.degreesToRadians(-45)));

    private final Transform3d robotToCam2 = new Transform3d(
                new Translation3d(Units.inchesToMeters(9.279), Units.inchesToMeters(9.52), Units.inchesToMeters(9.125)), // camera placement on robot
                new Rotation3d(0, Units.degreesToRadians(14), Units.degreesToRadians(45)));

    AprilTagFieldLayout aprilTagFieldLayout;    
    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    PhotonPoseEstimator poseEstimator1;
    PhotonPoseEstimator poseEstimator2;

    private boolean m_highCamDisabled = false;

    public AprilTagCamera() {
        init();
        PhotonCamera.setVersionCheckEnabled(false);
    }

    public void updateField2d(Field2d field) {
        var poseList = new ArrayList<Pose2d>();
        for (int i = 1; i <= 8; i++) {
            poseList.add(aprilTagFieldLayout.getTagPose(i).get().toPose2d());
        }

        field.getObject("April Tags").setPoses(poseList);
    }

    private void init() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        camList.add(new Pair<PhotonCamera, Transform3d>(rightLowCam, robotToCam1));
        poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, rightLowCam,
                robotToCam1);

        camList.add(new Pair<PhotonCamera, Transform3d>(leftLowCam, robotToCam2));
        poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, leftLowCam,
                        robotToCam2);
    }

    public void periodic() {
    }

    /**
     * 
     * @param prevEstimatedRobotPose
     * @return an EstimatedRobotPose which includes a Pose3d of the latest estimated
     *         pose (using the selected strategy) along with a double of the
     *         timestamp when the robot pose was estimated
     *         Use this to update drivetrain pose estimator
     */
    // public ArrayList<Optional<EstimatedRobotPose>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     ArrayList<Optional<EstimatedRobotPose>> estimatedGlobalPoses = new ArrayList<>();
    //     poseEstimator1.setReferencePose(prevEstimatedRobotPose);
    //     poseEstimator1.setLastPose(prevEstimatedRobotPose);
    //     estimatedGlobalPoses.add( poseEstimator1.update());
        
    //     poseEstimator2.setReferencePose(prevEstimatedRobotPose);
    //     poseEstimator2.setLastPose(prevEstimatedRobotPose);
    //     estimatedGlobalPoses.add( poseEstimator2.update());

    //     return estimatedGlobalPoses;
    // }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose1(Pose2d prevEstimatedRobotPose) {
        poseEstimator1.setReferencePose(prevEstimatedRobotPose);
        poseEstimator1.setLastPose(prevEstimatedRobotPose);
        return poseEstimator1.update();
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose2(Pose2d prevEstimatedRobotPose) {
        if (m_highCamDisabled) {
            return null;
        }
        poseEstimator2.setReferencePose(prevEstimatedRobotPose);
        poseEstimator2.setLastPose(prevEstimatedRobotPose);
        return poseEstimator2.update();
    }

    public void setHighCamDisabled(boolean disabled) {
        m_highCamDisabled = disabled;
    }

    // unfiltered view of camera
    public void toggleDriverMode() {
        if (rightLowCam.getDriverMode()) {
            rightLowCam.setDriverMode(false);
        }

        else {
            rightLowCam.setDriverMode(true);
        }
    }
}
