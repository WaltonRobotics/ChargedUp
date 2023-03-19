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
    public final PhotonCamera rightLowCam = new PhotonCamera("RightCornerLow");
    public final PhotonCamera leftLowCam = new PhotonCamera("LeftCornerLow");

    private final Transform3d rightLowCamToRobot = new Transform3d(
            new Translation3d(Units.inchesToMeters(9.52), Units.inchesToMeters(9.279), Units.inchesToMeters(8.845)),
            new Rotation3d(0, Units.degreesToRadians(14), Units.degreesToRadians(39.6)));

    private final Transform3d leftLowCamToRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(9.52), Units.inchesToMeters(9.279), Units.inchesToMeters(8.845)),
                new Rotation3d(0, Units.degreesToRadians(-14), Units.degreesToRadians(-39.6)));
    

    AprilTagFieldLayout aprilTagFieldLayout;    
    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    PhotonPoseEstimator leftLowPoseEstimator;
    PhotonPoseEstimator rightLowPoseEstimator;

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

        camList.add(new Pair<PhotonCamera, Transform3d>(rightLowCam, leftLowCamToRobot));
        leftLowPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, leftLowCam,
                leftLowCamToRobot);

        camList.add(new Pair<PhotonCamera, Transform3d>(leftLowCam, rightLowCamToRobot));
        rightLowPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, rightLowCam,
                        rightLowCamToRobot);
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

    public Optional<EstimatedRobotPose> leftLow_getEstPose(Pose2d prevEstimatedRobotPose) {
        leftLowPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        leftLowPoseEstimator.setLastPose(prevEstimatedRobotPose);
        return leftLowPoseEstimator.update();
    }
    
    public Optional<EstimatedRobotPose> rightLow_getEstPose(Pose2d prevEstimatedRobotPose) {
        rightLowPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        rightLowPoseEstimator.setLastPose(prevEstimatedRobotPose);
        return rightLowPoseEstimator.update();
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
