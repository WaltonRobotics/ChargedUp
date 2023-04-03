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

    private static boolean ignoreFrame(PhotonPipelineResult frame) {
        if (!frame.hasTargets() || frame.getTargets().size() > VisionK.MAX_FRAME_FIDS)
          return true;
    
        boolean possibleCombination = false;
        List<Integer> ids = frame.targets.stream().map(t -> t.getFiducialId()).toList();
        for (Set<Integer> possibleFIDCombo : VisionK.POSSIBLE_FRAME_FID_COMBOS) {
          possibleCombination = possibleFIDCombo.containsAll(ids);
          if (possibleCombination) break;
        }
        if (!possibleCombination) System.out.println("Ignoring frame with FIDs: " + ids);
        return !possibleCombination;
      }

    private final List<CameraEstimator> cameraEstimators = new ArrayList<>();

    private AprilTagFieldLayout fieldLayout;
    private ConcurrentLinkedQueue<VisionMeasurement> visionMeasurements = new ConcurrentLinkedQueue<>();
    private double lastDetection = 0;

    public VisionManager() {
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
    }

    private void logMeasurement(int tags, double avgDistance, double ambiguity, Pose3d est) {
        // TODO
    }

    private void findVisionMeasurements() {
        for (CameraEstimator cameraEstimator : cameraEstimators) {
          PhotonPipelineResult frame = cameraEstimator.camera().getLatestResult();
    
          // determine if result should be ignored
          if (cameraEstimator.dupeTracker().isDuplicate(frame) || ignoreFrame(frame)) continue;
    
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
    
          // System.out.println(
          //     String.format(
          //         "with %d tags at smallest distance %f and pose ambiguity factor %f, confidence
          // multiplier %f",
          //         estimation.targetsUsed.size(),
          //         smallestDistance,
          //         poseAmbiguityFactor,
          //         confidenceMultiplier));
          lastDetection = estimation.timestampSeconds;
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
