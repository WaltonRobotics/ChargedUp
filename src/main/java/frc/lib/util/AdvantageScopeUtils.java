package frc.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AdvantageScopeUtils {
    private AdvantageScopeUtils() {}

    // public static double

    public static double[] toDoubleArr(Pose3d pose) {
        var t = pose.getTranslation();
        var q = pose.getRotation().getQuaternion();
        return new double[] {
          t.getX(), t.getY(), t.getZ(),
          q.getW(), q.getX(), q.getY(), q.getZ()
        };
    }

    public static double[] toDoubleArr(Transform3d transform) {
      var t = transform.getTranslation();
      var q = transform.getRotation().getQuaternion();
      return new double[] {
        t.getX(), t.getY(), t.getZ(),
        q.getW(), q.getX(), q.getY(), q.getZ()
      };
  }
}
