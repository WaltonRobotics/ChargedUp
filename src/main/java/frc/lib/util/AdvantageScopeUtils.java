package frc.lib.util;

import edu.wpi.first.math.geometry.Pose3d;

public class AdvantageScopeUtils {
    private AdvantageScopeUtils() {}

    public static double[] arrFromPose3d(Pose3d pose) {
        var t = pose.getTranslation();
        var q = pose.getRotation().getQuaternion();
        return new double[] {
          t.getX(), t.getY(), t.getZ(),
          q.getW(), q.getX(), q.getY(), q.getZ()
        };
    }
}
