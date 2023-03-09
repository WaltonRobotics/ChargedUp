package frc.lib.util;

import edu.wpi.first.wpilibj.util.Color;

public class LedUtils {
    private LedUtils() {}

    public static Color fixColor(Color orig) {
        return new Color(orig.red, orig.blue, orig.green);
    }
}
