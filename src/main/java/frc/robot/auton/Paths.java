package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.SwerveK.kKinematics;

import java.util.Arrays;
import java.util.List;

public class Paths {
	private static Rotation2d rot2dDeg(double deg) {
		return Rotation2d.fromDegrees(deg);
	}

	// this is not needed for PPSwerve autons
	public static final TrajectoryConfig config = new TrajectoryConfig(
			kMaxSpeedMetersPerSecond,
			kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(kKinematics);
	public static final PathConstraints kPPConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
			kMaxAccelerationMetersPerSecondSquared);

	public static final class PPPaths {
		public static final PathPlannerTrajectory oneConePark = PathPlanner.loadPath("oneConePark",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory twoElement = PathPlanner.loadPath("twoElement",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory coneOneHalf = PathPlanner.loadPath("coneOneHalf", 
		kMaxSpeedMetersPerSecond,
		kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory oneCubePark = PathPlanner.loadPath("oneCubePark", 
		kMaxSpeedMetersPerSecond,
		kMaxAccelerationMetersPerSecondSquared);

		public static final PathPlannerTrajectory threePiece1 = PathPlanner.loadPath("threePiece1",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory threePiece2 = PathPlanner.loadPath("threePiece2",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory threePiece3 = PathPlanner.loadPath("threePiece3",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory threePiece4 = PathPlanner.loadPath("threePiece4",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory backPark = PathPlanner.loadPath("backPark",
				1.0,
				3);
	}

	public static final class PPAutoscoreClass {
		public static final PathConstraints pathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
				kMaxSpeedMetersPerSecond);

		public static List<PathPoint> notAPath = Arrays.asList(ReferencePoints.currentPoint);
		public static List<PathPoint> bumpy = Arrays.asList(ReferencePoints.bumper1, ReferencePoints.bumper2);
		public static List<PathPoint> notBumpy = Arrays.asList(ReferencePoints.notBumper1,
				ReferencePoints.notBumper2);
	}

	public static final class ReferencePoints {
		// position, heading (rotation), holonomic rotation
		public static AprilTagFieldLayout aprilTagFieldLayout;

		// tag poses
		public static final Pose2d tag1Pose = new Pose2d(new Translation2d(14.22, 1.05), rot2dDeg(0));
		public static final Pose2d tag2Pose = new Pose2d(new Translation2d(14.22, 2.30), rot2dDeg(0));
		public static final Pose2d tag3Pose = new Pose2d(new Translation2d(14.22, 4.47), rot2dDeg(0));
		public static final Pose2d tag4Pose = new Pose2d(new Translation2d(15.78, 6.7), rot2dDeg(0));
		public static final Pose2d tag5Pose = new Pose2d(new Translation2d(0.91, 6.7), rot2dDeg(180));
		public static final Pose2d tag6Pose = new Pose2d(new Translation2d(2, 4.37), rot2dDeg(180));
		public static final Pose2d tag7Pose = new Pose2d(new Translation2d(2, 2.7), rot2dDeg(180));
		public static final Pose2d tag8Pose = new Pose2d(new Translation2d(2, 1.05), rot2dDeg(180));

		// tag pathpoints
		public static PathPoint currentPoint;
		public static final PathPoint tag1 = new PathPoint(tag1Pose.getTranslation(), rot2dDeg(-90),
				tag1Pose.getRotation());
		public static final PathPoint tag2 = new PathPoint(tag2Pose.getTranslation(), rot2dDeg(-90),
				tag2Pose.getRotation());
		public static final PathPoint tag3 = new PathPoint(tag3Pose.getTranslation(), rot2dDeg(-90),
				tag3Pose.getRotation());
		public static final PathPoint tag4 = new PathPoint(tag4Pose.getTranslation(), rot2dDeg(-90),
				tag4Pose.getRotation());
		public static final PathPoint tag5 = new PathPoint(tag5Pose.getTranslation(), rot2dDeg(-90),
				tag5Pose.getRotation());
		public static final PathPoint tag6 = new PathPoint(tag6Pose.getTranslation(), rot2dDeg(-90),
				tag6Pose.getRotation());
		public static final PathPoint tag7 = new PathPoint(tag7Pose.getTranslation(), rot2dDeg(-90),
				tag7Pose.getRotation());
		public static final PathPoint tag8 = new PathPoint(tag8Pose.getTranslation(), rot2dDeg(-90),
				tag8Pose.getRotation());
		
		public static final Pose2d notBumper1Pose = new Pose2d(new Translation2d(5.84, 4.75), rot2dDeg(180));
		public static final Pose2d notBumper2Pose = new Pose2d(new Translation2d(2.5, 4.75), rot2dDeg(180));
		public static final Pose2d bumper1Pose = new Pose2d(new Translation2d(5.84, 0.75), rot2dDeg(180));
		public static final Pose2d bumper2Pose = new Pose2d(new Translation2d(3.26, 0.75), rot2dDeg(180));

		public static final PathPoint notBumper1 = new PathPoint(notBumper1Pose.getTranslation(), rot2dDeg(-165),
				notBumper1Pose.getRotation());
		public static final PathPoint notBumper2 = new PathPoint(notBumper2Pose.getTranslation(), rot2dDeg(0),
				notBumper2Pose.getRotation());
		public static final PathPoint bumper1 = new PathPoint(bumper1Pose.getTranslation(), rot2dDeg(180),
				bumper1Pose.getRotation(), 2);
		public static final PathPoint bumper2 = new PathPoint(bumper2Pose.getTranslation(), rot2dDeg(175),
				bumper2Pose.getRotation());

		public static final Pose2d oneConeInitial = new Pose2d(1.68, 5.00, Rotation2d.fromDegrees(180));
		public static final Pose2d oneConeWaypoint = new Pose2d(5.41, 5.00, Rotation2d.fromDegrees(180));
		public static final Pose2d oneConeEnd = new Pose2d(5.41, 2.91, Rotation2d.fromDegrees(180));

		public static PathPoint toPathPoint(Pose2d point) {
			return new PathPoint(point.getTranslation(), rot2dDeg(0), point.getRotation());
		}

		public static class ScoringPoints {
			public static final Pose2d cone1 =  
				new Pose2d(new Translation2d(1.77, 0.50), rot2dDeg(180));
			public static final Pose2d cube2 = 
				new Pose2d(new Translation2d(1.77, 1.06), rot2dDeg(180));
			public static final Pose2d cone3 = 
				new Pose2d(new Translation2d(1.77, 1.62), rot2dDeg(180));
			public static final Pose2d coopCone4 = 
				new Pose2d(new Translation2d(1.77, 2.18), rot2dDeg(180));
			public static final Pose2d coopCube5 = new Pose2d(new Translation2d(1.77, 2.74), rot2dDeg(180));
			public static final Pose2d coopCone6 = 
				new Pose2d(new Translation2d(1.77, 3.29), rot2dDeg(180));
			public static final Pose2d cone7 = 
				new Pose2d(new Translation2d(1.77, 3.85), rot2dDeg(180));
			public static final Pose2d cube8 = 
				new Pose2d(new Translation2d(1.77, 4.42), rot2dDeg(180));
			public static final Pose2d cone9 =
				new Pose2d(new Translation2d(1.77, 4.98), rot2dDeg(180));

			public static final Pose2d substationPose = new Pose2d(new Translation2d(15.9, 6.68), rot2dDeg(0));
			public static final Pose2d portalPose = new Pose2d(new Translation2d(13.63, 7.62), rot2dDeg(90));
		}
	}
}
