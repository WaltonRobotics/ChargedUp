package frc.robot.vision;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class AprilTagCamera {

    private final Transform3d rightLowRobotToCamera = new Transform3d(
        new Translation3d(Units.inchesToMeters(9.52), Units.inchesToMeters(-9.279), Units.inchesToMeters(8.845)),
        new Rotation3d(0, Units.degreesToRadians(-14), Units.degreesToRadians(39.6)));

    private final Transform3d leftLowRobotToCamera = new Transform3d(
        new Translation3d(Units.inchesToMeters(9.52), Units.inchesToMeters(9.279), Units.inchesToMeters(8.845)),
        new Rotation3d(0, Units.degreesToRadians(-14), Units.degreesToRadians(-50.5)));
    
    AprilTagFieldLayout aprilTagFieldLayout;

    // private final DoubleArrayPublisher log_leftRobotToCam, log_rightRobotToCam;

    public final PhotonRunner rightLowCamRunner, leftLowCamRunner;

    public AprilTagCamera() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        rightLowCamRunner = new PhotonRunner(
            "RightCornerLow",
            rightLowRobotToCamera,
            aprilTagFieldLayout,
            () -> new Pose2d()
        );

        leftLowCamRunner = new PhotonRunner(
            "LeftCornerLow",
            leftLowRobotToCamera,
            aprilTagFieldLayout,
            () -> new Pose2d()
        );

        PhotonCamera.setVersionCheckEnabled(false);
    }

    public void updateField2d(Field2d field) {
        var poseList = new ArrayList<Pose2d>();
        for (int i = 1; i <= 8; i++) {
            poseList.add(aprilTagFieldLayout.getTagPose(i).get().toPose2d());
        }

        field.getObject("April Tags").setPoses(poseList);
    }

    public void periodic() {
    }


    // unfiltered view of camera
    // public void setDriverMode(boolean driverMode) {
    //     leftLowCamRunner.setDriverMode(driverMode);
    //     rightLowCam.setDriverMode(driverMode);
    // }
}
