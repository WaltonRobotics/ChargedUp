package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionK;

public class VisionManager {

    private class DuplicateTracker {
        private double lastTimeStamp;
    
        public boolean isDuplicate(PhotonPipelineResult frame) {
          boolean isDuplicate = frame.getTimestampSeconds() == lastTimeStamp;
          lastTimeStamp = frame.getTimestampSeconds();
          return isDuplicate;
        }
      }

    record CameraEstimator(PhotonCamera camera, PhotonPoseEstimator estimator, DuplicateTracker dupeTracker) {}

    public static record VisionMeasurement(EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {}

    private static boolean ignoreFrame(PhotonPipelineResult frame, Set<Integer> allowedIds) {
        if (!frame.hasTargets() || frame.getTargets().size() > VisionK.MAX_FRAME_FIDS)
          return true;

        boolean possibleCombination = false;
        boolean allowedCombination = false;
        List<Integer> ids = frame.targets.stream().map(t -> t.getFiducialId()).toList();
        for (Set<Integer> possibleFIDCombo : VisionK.POSSIBLE_FRAME_FID_COMBOS) {
          possibleCombination = possibleFIDCombo.containsAll(ids);
          allowedCombination = allowedIds.containsAll(ids);
          if (possibleCombination && allowedCombination) break;
        }
        if (!possibleCombination) System.out.println("Ignoring frame with FIDs: " + ids);
        return !possibleCombination;
      }

    private final List<CameraEstimator> cameraEstimators = new ArrayList<>();

    private AprilTagFieldLayout fieldLayout;
    private ConcurrentLinkedQueue<VisionMeasurement> visionMeasurements = new ConcurrentLinkedQueue<>();

    public VisionManager() {
        double classInitBegin = Timer.getFPGATimestamp();
        System.out.println("[INIT] VisionManager Init Begin");
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            System.err.println("Failed to load field layout.");
            e.printStackTrace();
            return;
        }

        for (VisionK.VisionSource visionSource : VisionK.VISION_SOURCES) {
            var camera = new PhotonCamera(visionSource.name());
            var estimator =
                new PhotonPoseEstimator(
                    fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                    camera,
                    visionSource.robotToCamera());
            estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
            cameraEstimators.add(new CameraEstimator(camera, estimator, new DuplicateTracker()));
        }

        var thread = new Thread(() -> {
            if (fieldLayout == null) return;
            while (!Thread.currentThread().isInterrupted()) {
                this.findVisionMeasurements();
                try {
                    Thread.sleep(VisionK.THREAD_SLEEP_DURATION_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        thread.setDaemon(true);
        thread.start();

        double classInitElapsed = Timer.getFPGATimestamp() - classInitBegin;
		System.out.println("[INIT] VisionManager Init End: " + classInitElapsed + "s");
    }

    private void logMeasurement(int tags, double avgDistance, double ambiguity, Pose3d est) {
        // TODO
    }

    private void findVisionMeasurements() {
        boolean allianceBlue = DriverStation.getAlliance() == Alliance.Blue;
        boolean auton = DriverStation.isAutonomous();
        Set<Integer> allowedIds = auton ? 
            (allianceBlue ? VisionK.BLUE_TAG_FIDS : VisionK.RED_TAG_FIDS ) :
            VisionK.ALL_TAG_FIDS;

        for (CameraEstimator cameraEstimator : cameraEstimators) {
          PhotonPipelineResult frame = cameraEstimator.camera().getLatestResult();
    

          // determine if result should be ignored
          if (cameraEstimator.dupeTracker().isDuplicate(frame) || ignoreFrame(frame, allowedIds)) continue;

          // remove targets more than 6m away
          frame.targets.removeIf(tag -> {
            var t3d = tag.getBestCameraToTarget();
            return (t3d.getX() > 6.0 || t3d.getY() > 6.0);
          });
    
          var optEstimation = cameraEstimator.estimator().update(frame);
          if (optEstimation.isEmpty()) continue;
          var estimation = optEstimation.get();
    
          if (estimation.targetsUsed.size() == 1
              && (estimation.targetsUsed.get(0).getPoseAmbiguity() > VisionK.POSE_AMBIGUITY_CUTOFF
                  || estimation.targetsUsed.get(0).getPoseAmbiguity() == -1)) continue;
    
          double sumDistance = 0;
          for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();

            sumDistance +=
                Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
          }
          double avgDistance = sumDistance / estimation.targetsUsed.size();
    
          var deviation =
              VisionK.TAG_COUNT_DEVIATION_PARAMS
                  .get(
                      MathUtil.clamp(
                          estimation.targetsUsed.size() - 1,
                          0,
                          VisionK.TAG_COUNT_DEVIATION_PARAMS.size() - 1))
                  .computeDeviation(avgDistance);
    
          logMeasurement(
              estimation.targetsUsed.size(),
              avgDistance,
              estimation.targetsUsed.get(0).getPoseAmbiguity(),
              estimation.estimatedPose);
          visionMeasurements.add(new VisionMeasurement(estimation, deviation));
        }
      }

    public VisionMeasurement drainVisionMeasurement() {
        return visionMeasurements.poll();
    }
}
