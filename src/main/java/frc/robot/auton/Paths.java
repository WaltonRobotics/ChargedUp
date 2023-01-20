package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

import java.util.List;

import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;

public class Paths {
    public static final TrajectoryConfig config =
            new TrajectoryConfig(
                    kMaxSpeedMetersPerSecond,
                    kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);

    public static final Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(1, 0), new Translation2d(4, 0)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(4, 40, new Rotation2d(0)),
                    config);

    public static final class UTest{
        public static final PathPlannerTrajectory uPath =
                PathPlanner.loadPath("uPath",
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared);
    }

    public static final class StraightPath {
        public static final PathPlannerTrajectory straightPath =
                PathPlanner.loadPath("straightPath",
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared);
    }
}
