package frc.robot.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.logging.WaltLogger;
import frc.lib.logging.WaltLogger.DoubleLogger;


public class PhotonRunner {
    private final Notifier camNotifier = new Notifier(this::run);
    private final PhotonCamera camera;
    private final Transform3d robotToCamTransform;
    private final PhotonPoseEstimator poseEstimator;
    private final Supplier<Pose2d> robotPoseSupplier;

    private Optional<EstimatedRobotPose> latestEstPose = Optional.empty();

    private final DoubleLogger log_estTimeSec;

    public PhotonRunner(
        String cameraName,
        Transform3d robotToCam,
        AprilTagFieldLayout atfl,
        Supplier<Pose2d> robotPoseSup
    ) {
        camera = new PhotonCamera(cameraName);
        robotToCamTransform = robotToCam;
        poseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP, camera, robotToCamTransform);
        camNotifier.startPeriodic(0.02);
        robotPoseSupplier = robotPoseSup;

        final String entryPrefix = "PhotonRunner/" + cameraName + "/";

        log_estTimeSec = WaltLogger.logDouble(entryPrefix, "estTimeSec");
    }

    private void run() {
        var estBegin = Timer.getFPGATimestamp();
        latestEstPose = getEstimatedPose(robotPoseSupplier.get());
        var estElapsed = Timer.getFPGATimestamp() - estBegin;
        log_estTimeSec.accept(estElapsed);
    }

    private Optional<EstimatedRobotPose> getEstimatedPose(Pose2d prevEstRobotPose) {
        poseEstimator.setReferencePose(prevEstRobotPose);
        poseEstimator.setLastPose(prevEstRobotPose);
        return poseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getLatestEstimatedPose() {
        return latestEstPose;
    }
}
