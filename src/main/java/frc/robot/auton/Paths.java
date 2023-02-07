package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.SwerveK.kKinematics;

import java.util.List;

public class Paths {
	// this is not needed for PPSwerve autons
	public static final TrajectoryConfig config = new TrajectoryConfig(
			kMaxSpeedMetersPerSecond,
			kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(kKinematics);
	public static final PathConstraints kPPConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
			kMaxAccelerationMetersPerSecondSquared);

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
		public static final List<PathPlannerTrajectory> twoPiece = PathPlanner.loadPathGroup("banks3", false,
				kPPConstraints);
		public static final PathPlannerTrajectory twoPieceBalance = PathPlanner.loadPath("twoPieceBalance", 
				kMaxSpeedMetersPerSecond, 
				kMaxAccelerationMetersPerSecondSquared);
	}

	public static final class ReferencePoints{
		//position, heading (rotation), holonomic rotation
		public static final PathPoint tag1 = new PathPoint(new Translation2d(14.70, 0.5), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(90));
		public static final PathPoint tag2 = new PathPoint(new Translation2d(14.80, 4.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0));
		public static final PathPoint tag5 = new PathPoint(new Translation2d(0.91, 6.70), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(-180));
		public static final PathPoint tag6 = new PathPoint(new Translation2d(1.77, 4.41), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(-180));
		public static final PathPoint tag7 = new PathPoint(new Translation2d(1.77, 2.70), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(-180));
		public static final PathPoint tag8 = new PathPoint(new Translation2d(1.77, 1.05), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(-180));
		public static final PathPoint redRightOut = new PathPoint(new Translation2d(11.15, 4.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(180));
		public static final PathPoint redRightIn = new PathPoint(new Translation2d(14.70, 4.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0));
		public static final PathPoint blueRightOut = new PathPoint(new Translation2d(6.15, 1.00), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
		public static final PathPoint blueRightIn = new PathPoint(new Translation2d(2.50, 1.00), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
		public static final PathPoint blueLeftOut = new PathPoint(new Translation2d(6.15, 4.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
		public static final PathPoint blueLeftIn = new PathPoint(new Translation2d(2.50, 4.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
	}
}
