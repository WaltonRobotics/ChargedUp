package com.pathplanner.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class PathPointAccessor {

    public static Pose2d poseFromPathPointHolo(PathPoint pp) {
        return new Pose2d(pp.position, pp.holonomicRotation);
    }

    public static Pose2d poseFromPathPointHeading(PathPoint pp) {
        return new Pose2d(pp.position, pp.heading);
    }
}
