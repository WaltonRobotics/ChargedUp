package frc.robot.vision;
//TODO: measure cameras
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import frc.lib.vision.EstimatedRobotPose;
// import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import frc.lib.vision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator;
import frc.lib.vision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class AprilTagCamera {
    public final PhotonCamera highCam = new PhotonCamera("LeftHighCam");
    public final PhotonCamera lowCam = new PhotonCamera("LeftLowCam"); // TODO: name the camera (will do when we have the actual camera)
    // distance from robot to camera
    private final Transform3d robotToCam1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(1.25), Units.inchesToMeters(42.5)), // camera placement on robot
            new Rotation3d(0, Units.degreesToRadians(0), 0));

    private final Transform3d robotToCam2 = new Transform3d(
                new Translation3d(Units.inchesToMeters(9.5), Units.inchesToMeters(8.183), Units.inchesToMeters(7.25)), // camera placement on robot
                new Rotation3d(0, Units.degreesToRadians(14), 0));

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

        camList.add(new Pair<PhotonCamera, Transform3d>(highCam, robotToCam1));
        poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, highCam,
                robotToCam1);

        camList.add(new Pair<PhotonCamera, Transform3d>(lowCam, robotToCam2));
        poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, lowCam,
                        robotToCam2);
    }

    public void periodic() {
        //reverses based on PathPlanner coordinates
		if(DriverStation.getAlliance() == Alliance.Blue){
			poseEstimator1.getFieldTags().setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            poseEstimator2.getFieldTags().setOrigin(OriginPosition.kBlueAllianceWallRightSide);
		}
		else{
			poseEstimator1.getFieldTags().setOrigin(OriginPosition.kRedAllianceWallRightSide);
            poseEstimator2.getFieldTags().setOrigin(OriginPosition.kRedAllianceWallRightSide);
		}
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
        poseEstimator2.setReferencePose(prevEstimatedRobotPose);
        poseEstimator2.setLastPose(prevEstimatedRobotPose);
        return poseEstimator2.update();
    }

    public void setHighCamDisabled(boolean disabled) {
        m_highCamDisabled = disabled;
    }

    // unfiltered view of camera
    public void toggleDriverMode() {
        if (highCam.getDriverMode()) {
            highCam.setDriverMode(false);
        }

        else {
            highCam.setDriverMode(true);
        }
    }

    // public Optional<PhotonPipelineResult> getLatestResult1() {
    //     var result1 = highCam.getLatestResult();
    //     return result1 != null ? Optional.of(result1) : Optional.empty();
    // }

    // public Optional<PhotonPipelineResult> getLatestResult2() {
    //     var result2 = lowCam.getLatestResult();

    //     return result2 != null && !m_highCamDisabled ? Optional.of(result2) : Optional.empty();
    // }

    // public Optional<PhotonTrackedTarget> getBestTarget1() {
    //     var latestOpt = getLatestResult1();
    //     if (latestOpt.isPresent()) {
    //         if (latestOpt.get().hasTargets()) {
    //             return Optional.of(latestOpt.get().getBestTarget());
    //         }
    //     }

    //     return Optional.empty();
    // }

    // public Optional<PhotonTrackedTarget> getBestTarget2() {
    //     var latestOpt = getLatestResult2();
    //     if (latestOpt.isPresent()) {
    //         if (latestOpt.get().hasTargets()) {
    //             return Optional.of(latestOpt.get().getBestTarget());
    //         }
    //     }

    //     return Optional.empty();
    // }

    // public void updateReferencePose(Pose2d poseMeters) {
    //     poseEstimator1.setReferencePose(poseMeters);
    //     poseEstimator2.setReferencePose(poseMeters);
    // }
}
