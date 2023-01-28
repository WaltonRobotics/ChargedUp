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
        public static final TrajectoryConfig config = new TrajectoryConfig(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.SwerveK.swerveKinematics);
        
        public static final Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 0), new Translation2d(4, 0)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(4, 40, new Rotation2d(0)),
                        config);  
        public static final Trajectory oneMeterForward = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(1, 0, new Rotation2d(0)),
                        config);  
        
        public static final Trajectory rotate90 = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        new Pose2d(1, 0, Rotation2d.fromDegrees(90)),
                        config); 
                
        public static final Trajectory moveDiagonal = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        new Pose2d(Math.sqrt(2)/2, Math.sqrt(2)/2, new Rotation2d(0)),
                        config); 

        //TODO: Revert speed constants back
        public static final class PPPaths {
                public static final PathPlannerTrajectory oneMeter = PathPlanner.loadPath("oneMeter",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
                public static final PathPlannerTrajectory straightPath = PathPlanner.loadPath("straightPath",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
                public static final PathPlannerTrajectory diagonal = PathPlanner.loadPath("diagonal",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
                public static final PathPlannerTrajectory rotate = PathPlanner.loadPath("rotate",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
                public static final PathPlannerTrajectory rotateMove = PathPlanner.loadPath("rotateMove",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
                public static final PathPlannerTrajectory threePiece = PathPlanner.loadPath("threePiece2",
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared);
        }
}
