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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
		public static final PathPlannerTrajectory straightBack = PathPlanner.loadPath("straightBack",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory oneConeOneCube = PathPlanner.loadPath("oneConeOneCube",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory testRot = PathPlanner.loadPath("testRot",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory oneConePark = PathPlanner.loadPath("oneConePark",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory oneConeParkEvents = PathPlanner.loadPath("oneConeParkEvents",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory oneCubeOneCone = PathPlanner.loadPath("oneCubeOneCone",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory twoConeOneCube = PathPlanner.loadPath("twoConeOneCube",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory cone1 = PathPlanner.loadPath(
				DriverStation.getAlliance().equals(Alliance.Red) ?
				"redCone1" :
				"redCone9",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory cube2 = PathPlanner.loadPath(
				DriverStation.getAlliance().equals(Alliance.Red) ?
				"redCube2" :
				"redCube8",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory cone3 = PathPlanner.loadPath(
				DriverStation.getAlliance().equals(Alliance.Red) ?
				"redCone3" :
				"redCone7",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory coopCone4 = PathPlanner.loadPath(
				DriverStation.getAlliance().equals(Alliance.Red) ?
				"redCoopCone4" :
				"redCoopCone6",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory coopCube5 = PathPlanner.loadPath("redCoopCube5",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory coopCone6 = PathPlanner.loadPath(
				DriverStation.getAlliance().equals(Alliance.Red) ?
				"redCoopCone6" :
				"redCoopCone4",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory cone7 = PathPlanner.loadPath(
				DriverStation.getAlliance().equals(Alliance.Red) ?
				"redCone7" :
				"redCone3",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory cube8 = PathPlanner.loadPath(
				DriverStation.getAlliance().equals(Alliance.Red) ?
				"redCube8" :
				"redCube2",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
		public static final PathPlannerTrajectory cone9 = PathPlanner.loadPath(
				DriverStation.getAlliance().equals(Alliance.Red) ?
				"redCone9" :
				"redCone1",
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared);
	}

	public static final class PPAutoscoreClass {
		public static final PathConstraints pathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
				kMaxSpeedMetersPerSecond);

		public static List<PathPoint> notAPath = Arrays.asList(ReferencePoints.currentPoint);
		// public static List<PathPoint> redBumpy =
		// Arrays.asList(ReferencePoints.redLeft1, ReferencePoints.redLeft2);
		// public static List<PathPoint> redNotBumpy =
		// Arrays.asList(ReferencePoints.redRight1, ReferencePoints.redRight2);
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

		// poses of pathpoints to get to community
		// public static final Pose2d redRight1Pose = new Pose2d(new
		// Translation2d(10.25, 4.65), rot2dDeg(0));
		// public static final Pose2d redRight2Pose = new Pose2d(new
		// Translation2d(13.28, 4.65), rot2dDeg(0));
		// public static final Pose2d redLeft1Pose = new Pose2d(new Translation2d(10.25,
		// 0.75), rot2dDeg(0));
		// public static final Pose2d redLeft2Pose = new Pose2d(new Translation2d(13.28,
		// 0.75), rot2dDeg(0));
		public static final Pose2d notBumper1Pose = new Pose2d(new Translation2d(5.84, 4.75), rot2dDeg(180));
		public static final Pose2d notBumper2Pose = new Pose2d(new Translation2d(2.5, 4.75), rot2dDeg(180));
		public static final Pose2d bumper1Pose = new Pose2d(new Translation2d(5.84, 0.75), rot2dDeg(180));
		public static final Pose2d bumper2Pose = new Pose2d(new Translation2d(3.26, 0.75), rot2dDeg(180));

		// pathpoints to get to community
		// public static final PathPoint redRight1 = new
		// PathPoint(redRight1Pose.getTranslation(), rot2dDeg(0),
		// redRight1Pose.getRotation());
		// public static final PathPoint redRight2 = new
		// PathPoint(redRight2Pose.getTranslation(), rot2dDeg(2.42),
		// redRight2Pose.getRotation());
		// // public static final PathPoint redRight3 = new PathPoint(new
		// Translation2d(13.75, 4.75), rot2dDeg(0),rot2dDeg(0));
		// public static final PathPoint redLeft1 = new
		// PathPoint(redLeft1Pose.getTranslation(), rot2dDeg(0),
		// redLeft1Pose.getRotation(), 3);
		// public static final PathPoint redLeft2 = new
		// PathPoint(redLeft2Pose.getTranslation(), rot2dDeg(2.42),
		// redLeft2Pose.getRotation(), 2);
		// public static final PathPoint redLeft3 = new PathPoint(new
		// Translation2d(13.180, 0.75), rot2dDeg(0),rot2dDeg(0));
		public static final PathPoint notBumper1 = new PathPoint(notBumper1Pose.getTranslation(), rot2dDeg(180),
				notBumper1Pose.getRotation());
		public static final PathPoint notBumper2 = new PathPoint(notBumper2Pose.getTranslation(), rot2dDeg(90),
				notBumper2Pose.getRotation());
		// public static final PathPoint blueLeft3 = new PathPoint(new
		// Translation2d(2.45, 4.75), rot2dDeg(180),rot2dDeg(180));
		public static final PathPoint bumper1 = new PathPoint(bumper1Pose.getTranslation(), rot2dDeg(180),
				bumper1Pose.getRotation(), 2);
		public static final PathPoint bumper2 = new PathPoint(bumper2Pose.getTranslation(), rot2dDeg(175),
				bumper2Pose.getRotation());

		// public static final PathPoint blueRight3 = new PathPoint(new
		// Translation2d(2.45, 0.75), rot2dDeg(180),rot2dDeg(180));
		public static PathPoint toPathPoint(Pose2d point) {
			return new PathPoint(point.getTranslation(), rot2dDeg(0), point.getRotation());
		}

		public static class ScoringPoints {
			// public static final Pose2d redCone1 = new Pose2d(new Translation2d(14.76,
			// 0.50), rot2dDeg(0));
			// public static final Pose2d redCube2 = new Pose2d(new Translation2d(14.76,
			// 1.06), rot2dDeg(0));
			// public static final Pose2d redCone3 = new Pose2d(new Translation2d(14.76,
			// 1.62), rot2dDeg(0));
			// public static final Pose2d redCoopCone4 = new Pose2d(new Translation2d(14.76,
			// 2.18), rot2dDeg(0));
			// public static final Pose2d redCoopCube5 = new Pose2d(new Translation2d(14.76,
			// 2.74), rot2dDeg(0));
			// public static final Pose2d redCoopCone6 = new Pose2d(new Translation2d(14.76,
			// 3.29), rot2dDeg(0));
			// public static final Pose2d redCone7 = new Pose2d(new Translation2d(14.76,
			// 3.85), rot2dDeg(0));
			// public static final Pose2d redCube8 = new Pose2d(new Translation2d(14.76,
			// 4.42), rot2dDeg(0));
			// public static final Pose2d redCone9 = new Pose2d(new Translation2d(14.76,
			// 4.98), rot2dDeg(0));

			// public static Pose2d[] redScoringPoints = {
			// redCone1, redCube2, redCone3, redCoopCone4, redCoopCube5, redCoopCone6,
			// redCone7, redCube8, redCone9
			// };

			public static final PathPoint cone1 =  
				new PathPoint(new Translation2d(1.77, 0.50), rot2dDeg(-90), rot2dDeg(180));
			public static final PathPoint cube2 = 
				new PathPoint(new Translation2d(1.77, 1.06), rot2dDeg(-90), rot2dDeg(180));
			public static final PathPoint cone3 = 
				new PathPoint(new Translation2d(1.77, 1.62), rot2dDeg(-90), rot2dDeg(180));
			public static final PathPoint coopCone4 = 
				new PathPoint(new Translation2d(1.77, 2.18), rot2dDeg(-90), rot2dDeg(180));
			public static final PathPoint coopCube5 = new PathPoint(new Translation2d(1.77, 2.74), rot2dDeg(90), rot2dDeg(180));
			public static final PathPoint coopCone6 = 
				new PathPoint(new Translation2d(1.77, 3.29), rot2dDeg(-90), rot2dDeg(180));
			public static final PathPoint cone7 = 
				new PathPoint(new Translation2d(1.77, 3.85), rot2dDeg(-90), rot2dDeg(180));
			public static final PathPoint cube8 = 
				new PathPoint(new Translation2d(1.77, 4.42), rot2dDeg(-90), rot2dDeg(180));
			public static final PathPoint cone9 =
				new PathPoint(new Translation2d(1.77, 4.98), rot2dDeg(-90), rot2dDeg(180));

			public static final Pose2d substationPose = new Pose2d(new Translation2d(15.9, 6.68), rot2dDeg(0));
			public static final Pose2d portalPose = new Pose2d(new Translation2d(13.63, 7.62), rot2dDeg(90));

			// public static Pose2d[] scoringPoints = {
			// 		cone1, cube2, cone3, coopCone4, coopCube5, coopCone6, cone7, cube8,
			// 		cone9,
			// 		substationPose, portalPose
			// };

		}

		// public enum ScoringPlaces {
		// 	CONE_1(ScoringPoints.cone1, 0),
		// 	CUBE_1(ScoringPoints.cube2, 1),
		// 	CONE_2(ScoringPoints.cone3, 0),
		// 	COOP_CONE_1(ScoringPoints.coopCone4, 0),
		// 	COOP_CUBE_1(ScoringPoints.coopCube5, 1),
		// 	COOP_CONE_2(ScoringPoints.coopCone6, 0),
		// 	CONE_3(ScoringPoints.cone7, 0),
		// 	CUBE_2(ScoringPoints.cube8, 1),
		// 	CONE_4(ScoringPoints.cone9, 0);

		// 	// public Pose2d redPt;
		// 	public Pose2d pt;
		// 	public int coneOrCube; // 0 is cone, 1 is cube

		// 	private ScoringPlaces(Pose2d pt, int coneOrCube) {
		// 		// this.redPt = redPt;
		// 		this.pt = pt;
		// 		this.coneOrCube = coneOrCube;
		// 	}
		// }
	} 

	// public static class ScoringOptionsRed {
	// public enum ScoringOptionRed {
	// CONE_1(ReferencePoints.ScoringPoints.redCone1, true),
	// CUBE_1(ReferencePoints.ScoringPoints.redCube2, false),
	// CONE_2(ReferencePoints.ScoringPoints.redCone3, true),
	// COOP_CONE_1(ReferencePoints.ScoringPoints.redCoopCone4, true),
	// COOP_CUBE_1(ReferencePoints.ScoringPoints.redCoopCube5, false),
	// COOP_CONE_2(ReferencePoints.ScoringPoints.redCoopCone6, true),
	// CONE_3(ReferencePoints.ScoringPoints.redCone7, true),
	// CUBE_2(ReferencePoints.ScoringPoints.redCube8, false),
	// CONE_4(ReferencePoints.ScoringPoints.redCone9, true);

	// public final Pose2d m_scoringPlace;
	// public final boolean m_cone;
	// private ScoringOptionRed(Pose2d scoringPlace, boolean cone) {
	// m_scoringPlace = scoringPlace;
	// m_cone = cone;
	// }
	// }

	// public enum ScoringOption {
	// 	CONE_1(ReferencePoints.ScoringPoints.cone1, true),
	// 	CUBE_1(ReferencePoints.ScoringPoints.cube2, false),
	// 	CONE_2(ReferencePoints.ScoringPoints.cone3, true),
	// 	COOP_CONE_1(ReferencePoints.ScoringPoints.coopCone4, true),
	// 	COOP_CUBE_1(ReferencePoints.ScoringPoints.coopCube5, false),
	// 	COOP_CONE_2(ReferencePoints.ScoringPoints.coopCone6, true),
	// 	CONE_3(ReferencePoints.ScoringPoints.cone7, true),
	// 	CUBE_2(ReferencePoints.ScoringPoints.cube8, false),
	// 	CONE_4(ReferencePoints.ScoringPoints.cone9, true);

	// 	public final Pose2d m_scoringPlace;
	// 	public final boolean m_cone;

	// 	private ScoringOption(Pose2d scoringPlace, boolean cone) {
	// 		m_scoringPlace = scoringPlace;
	// 		m_cone = cone;
	// 	}
	// }
}
