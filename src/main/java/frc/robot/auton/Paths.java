package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;

import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;

public class Paths {
        // this is not needed for PPSwerve autons
        public static final TrajectoryConfig config = new TrajectoryConfig(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveK.swerveKinematics);

        public static final class PPPaths {
                public static final PathPlannerTrajectory oneMeter = PathPlanner.loadPath("oneMeter",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
                public static final PathPlannerTrajectory threePiece2 = PathPlanner.loadPath("threePiece2",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
                public static final PathPlannerTrajectory threePiece3 = PathPlanner.loadPath("threePiece3",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
        }
}
