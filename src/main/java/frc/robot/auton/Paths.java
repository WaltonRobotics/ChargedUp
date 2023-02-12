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

import java.util.Arrays;
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
		public static final PathPlannerTrajectory onePiece = PathPlanner.loadPath("onePiece", 
				kMaxSpeedMetersPerSecond, 
				kMaxAccelerationMetersPerSecondSquared);
	}

	public static final class PPAutoscoreClass {
		public static final PathConstraints pathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond, kMaxSpeedMetersPerSecond);
		
		public static List<PathPoint> notAPath = Arrays.asList(ReferencePoints.currentPoint);
		public static List<PathPoint> redBumpy = Arrays.asList(ReferencePoints.redLeft1, ReferencePoints.redLeft2, ReferencePoints.redLeft3);
		public static List<PathPoint> redNotBumpy = Arrays.asList(ReferencePoints.redRight1, ReferencePoints.redRight2, ReferencePoints.redRight3);
		public static List<PathPoint> blueBumpy = Arrays.asList(ReferencePoints.blueRight1, ReferencePoints.blueRight2, ReferencePoints.blueRight3);
		public static List<PathPoint> blueNotBumpy = Arrays.asList(ReferencePoints.blueLeft1, ReferencePoints.blueLeft2, ReferencePoints.blueLeft3);
	}

	public static final class ReferencePoints{
		//position, heading (rotation), holonomic rotation
		public static PathPoint currentPoint;
		public static final PathPoint tag1 = new PathPoint(new Translation2d(14.25, 1.05), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(0));
		public static final PathPoint tag2 = new PathPoint(new Translation2d(14.25, 2.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(0));
		public static final PathPoint tag3 = new PathPoint(new Translation2d(14.25, 4.47), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(0));
		public static final PathPoint tag4 = new PathPoint(new Translation2d(15.78, 6.70), Rotation2d.fromDegrees(90),Rotation2d.fromDegrees(0));
		public static final PathPoint tag5 = new PathPoint(new Translation2d(0.91, 6.70), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(-180));
		public static final PathPoint tag6 = new PathPoint(new Translation2d(2, 4.37), Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(-180));
		public static final PathPoint tag7 = new PathPoint(new Translation2d(2, 2.70), Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(-180));
		public static final PathPoint tag8 = new PathPoint(new Translation2d(2, 1.05), Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(-180));

		public static final PathPoint redRight1 = new PathPoint(new Translation2d(10.25, 4.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0));
		public static final PathPoint redRight2 = new PathPoint(new Translation2d(12.52, 4.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0));
		public static final PathPoint redRight3 = new PathPoint(new Translation2d(13.75, 4.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0));
		public static final PathPoint redLeft1 = new PathPoint(new Translation2d(10.25, 0.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0), 3);
		public static final PathPoint redLeft2 = new PathPoint(new Translation2d(12.52, 0.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0), 2);
		public static final PathPoint redLeft3 = new PathPoint(new Translation2d(13.180, 0.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0));
		public static final PathPoint blueLeft1 = new PathPoint(new Translation2d(5.84, 4.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
		public static final PathPoint blueLeft2 = new PathPoint(new Translation2d(3.26, 4.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
		public static final PathPoint blueLeft3 = new PathPoint(new Translation2d(2.45, 4.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
		public static final PathPoint blueRight1 = new PathPoint(new Translation2d(5.84, 0.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180), 2);
		public static final PathPoint blueRight2 = new PathPoint(new Translation2d(3.26, 0.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
		public static final PathPoint blueRight3 = new PathPoint(new Translation2d(2.45, 0.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
	}
}
