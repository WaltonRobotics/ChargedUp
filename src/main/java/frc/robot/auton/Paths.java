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

import java.util.ArrayList;
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
		public static List<PathPoint> redBumpy = Arrays.asList(ReferencePoints.redLeft1, ReferencePoints.redLeft2);
		public static List<PathPoint> redNotBumpy = Arrays.asList(ReferencePoints.redRight1, ReferencePoints.redRight2);
		public static List<PathPoint> blueBumpy = Arrays.asList(ReferencePoints.blueRight1, ReferencePoints.blueRight2);
		public static List<PathPoint> blueNotBumpy = Arrays.asList(ReferencePoints.blueLeft1, ReferencePoints.blueLeft2);
	}

	public static final class ReferencePoints{
		//position, heading (rotation), holonomic rotation
		public static AprilTagFieldLayout aprilTagFieldLayout;

		// tag poses
		public static final Pose2d tag1Pose = new Pose2d(new Translation2d(14.22, 1.05), Rotation2d.fromDegrees(0));
		public static final Pose2d tag2Pose = new Pose2d(new Translation2d(14.22, 2.30), Rotation2d.fromDegrees(0));
		public static final Pose2d tag3Pose = new Pose2d(new Translation2d(14.22, 4.47), Rotation2d.fromDegrees(0));
		public static final Pose2d tag4Pose = new Pose2d(new Translation2d(15.78, 6.7), Rotation2d.fromDegrees(0));
		public static final Pose2d tag5Pose = new Pose2d(new Translation2d(0.91, 6.7), Rotation2d.fromDegrees(180));
		public static final Pose2d tag6Pose = new Pose2d(new Translation2d(2, 4.37), Rotation2d.fromDegrees(180));
		public static final Pose2d tag7Pose = new Pose2d(new Translation2d(2, 2.7), Rotation2d.fromDegrees(180));
		public static final Pose2d tag8Pose = new Pose2d(new Translation2d(2, 1.05), Rotation2d.fromDegrees(180));

		// tag pathpoints
		public static PathPoint currentPoint;
		public static final PathPoint tag1 = new PathPoint(tag1Pose.getTranslation(), Rotation2d.fromDegrees(-90),tag1Pose.getRotation());
		public static final PathPoint tag2 = new PathPoint(tag2Pose.getTranslation(), Rotation2d.fromDegrees(-90),tag2Pose.getRotation());
		public static final PathPoint tag3 = new PathPoint(tag3Pose.getTranslation(), Rotation2d.fromDegrees(-90),tag3Pose.getRotation());
		public static final PathPoint tag4 = new PathPoint(tag4Pose.getTranslation(), Rotation2d.fromDegrees(-90),tag4Pose.getRotation());
		public static final PathPoint tag5 = new PathPoint(tag5Pose.getTranslation(), Rotation2d.fromDegrees(-90),tag5Pose.getRotation());
		public static final PathPoint tag6 = new PathPoint(tag6Pose.getTranslation(), Rotation2d.fromDegrees(-90),tag6Pose.getRotation());
		public static final PathPoint tag7 = new PathPoint(tag7Pose.getTranslation(), Rotation2d.fromDegrees(-90),tag7Pose.getRotation());
		public static final PathPoint tag8 = new PathPoint(tag8Pose.getTranslation(), Rotation2d.fromDegrees(-90),tag8Pose.getRotation());

		// red scoring
		public static final Pose2d redCone1 = new Pose2d(new Translation2d(14.76, 0.50), Rotation2d.fromDegrees(0));
		public static final Pose2d redCube1 = new Pose2d(new Translation2d(14.76, 1.06), Rotation2d.fromDegrees(0));
		public static final Pose2d redCone2 = new Pose2d(new Translation2d(14.76, 1.62), Rotation2d.fromDegrees(0));
		public static final Pose2d redCoopCone1 = new Pose2d(new Translation2d(14.76, 2.18), Rotation2d.fromDegrees(0));
		public static final Pose2d redCoopCube1 = new Pose2d(new Translation2d(14.76, 2.74), Rotation2d.fromDegrees(0));
		public static final Pose2d redCoopCone2 = new Pose2d(new Translation2d(14.76, 3.29), Rotation2d.fromDegrees(0));
		public static final Pose2d redCone3 = new Pose2d(new Translation2d(14.76, 3.85), Rotation2d.fromDegrees(0));
		public static final Pose2d redCube2 = new Pose2d(new Translation2d(14.76, 4.42), Rotation2d.fromDegrees(0));
		public static final Pose2d redCone4 = new Pose2d(new Translation2d(14.76, 4.98), Rotation2d.fromDegrees(0));

		// blue scoring
		public static final Pose2d blueCone1 = new Pose2d(new Translation2d(1.77, 0.50), Rotation2d.fromDegrees(180));
		public static final Pose2d blueCube1 = new Pose2d(new Translation2d(1.77, 1.06), Rotation2d.fromDegrees(180));
		public static final Pose2d blueCone2 = new Pose2d(new Translation2d(1.77, 1.62), Rotation2d.fromDegrees(180));
		public static final Pose2d blueCoopCone1 = new Pose2d(new Translation2d(1.77, 2.18), Rotation2d.fromDegrees(180));
		public static final Pose2d blueCoopCube1 = new Pose2d(new Translation2d(1.77, 2.74), Rotation2d.fromDegrees(180));
		public static final Pose2d blueCoopCone2 = new Pose2d(new Translation2d(1.77, 3.29), Rotation2d.fromDegrees(180));
		public static final Pose2d blueCone3 = new Pose2d(new Translation2d(1.77, 3.85), Rotation2d.fromDegrees(180));
		public static final Pose2d blueCube2 = new Pose2d(new Translation2d(1.77, 4.42), Rotation2d.fromDegrees(180));
		public static final Pose2d blueCone4 = new Pose2d(new Translation2d(1.77, 4.98), Rotation2d.fromDegrees(180));

		// poses of pathpoints to get to community
		public static final Pose2d redRight1Pose = new Pose2d(new Translation2d(10.25, 4.65), Rotation2d.fromDegrees(0));
		public static final Pose2d redRight2Pose = new Pose2d(new Translation2d(13.28, 4.65), Rotation2d.fromDegrees(0));
		public static final Pose2d redLeft1Pose = new Pose2d(new Translation2d(10.25, 0.75), Rotation2d.fromDegrees(0));
		public static final Pose2d redLeft2Pose = new Pose2d(new Translation2d(13.28, 0.75), Rotation2d.fromDegrees(0));
		public static final Pose2d blueLeft1Pose = new Pose2d(new Translation2d(5.84, 4.75), Rotation2d.fromDegrees(180));
		public static final Pose2d blueLeft2Pose = new Pose2d(new Translation2d(3.36, 4.75), Rotation2d.fromDegrees(180));
		public static final Pose2d blueRight1Pose = new Pose2d(new Translation2d(5.84, 0.75), Rotation2d.fromDegrees(180));
		public static final Pose2d blueRight2Pose = new Pose2d(new Translation2d(3.26, 0.75), Rotation2d.fromDegrees(180));

		// pathpoints to get to community
		public static final PathPoint redRight1 = new PathPoint(redRight1Pose.getTranslation(), Rotation2d.fromDegrees(0), redRight1Pose.getRotation());
		public static final PathPoint redRight2 = new PathPoint(redRight2Pose.getTranslation(), Rotation2d.fromDegrees(2.42), redRight2Pose.getRotation());
		// public static final PathPoint redRight3 = new PathPoint(new Translation2d(13.75, 4.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0));
		public static final PathPoint redLeft1 = new PathPoint(redLeft1Pose.getTranslation(), Rotation2d.fromDegrees(0), redLeft1Pose.getRotation(), 3);
		public static final PathPoint redLeft2 = new PathPoint(redLeft2Pose.getTranslation(), Rotation2d.fromDegrees(2.42), redLeft2Pose.getRotation(), 2);
		// public static final PathPoint redLeft3 = new PathPoint(new Translation2d(13.180, 0.75), Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(0));
		public static final PathPoint blueLeft1 = new PathPoint(blueLeft1Pose.getTranslation(), Rotation2d.fromDegrees(180), blueLeft1Pose.getRotation());
		public static final PathPoint blueLeft2 = new PathPoint(blueLeft2Pose.getTranslation(), Rotation2d.fromDegrees(175), blueLeft2Pose.getRotation());
		// public static final PathPoint blueLeft3 = new PathPoint(new Translation2d(2.45, 4.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
		public static final PathPoint blueRight1 = new PathPoint(blueRight1Pose.getTranslation(), Rotation2d.fromDegrees(180),blueRight1Pose.getRotation(), 2);
		public static final PathPoint blueRight2 = new PathPoint(blueRight2Pose.getTranslation(), Rotation2d.fromDegrees(175), blueRight2Pose.getRotation());
		// public static final PathPoint blueRight3 = new PathPoint(new Translation2d(2.45, 0.75), Rotation2d.fromDegrees(180),Rotation2d.fromDegrees(180));
	}

	public enum ScoringOptionRed {
		CONE_1(ReferencePoints.redCone1),
		CUBE_1(ReferencePoints.redCube1),
		CONE_2(ReferencePoints.redCone2),
		COOP_CONE_1(ReferencePoints.redCoopCone1),
		COOP_CUBE_1(ReferencePoints.redCoopCube1),
		COOP_CONE_2(ReferencePoints.redCoopCone2),
		RED_CONE_3(ReferencePoints.redCone3),
		RED_CUBE_2(ReferencePoints.redCube2),
		RED_CONE_4(ReferencePoints.redCone4);

		public final Pose2d m_scoringPlace; 
		private ScoringOptionRed(Pose2d scoringPlace) {
			m_scoringPlace = scoringPlace;
		}
	}

	public enum ScoringOptionBlue {
		CONE_1(ReferencePoints.blueCone1),
		CUBE_1(ReferencePoints.blueCube1),
		CONE_2(ReferencePoints.blueCone2),
		COOP_CONE_1(ReferencePoints.blueCoopCone1),
		COOP_CUBE_1(ReferencePoints.blueCoopCube1),
		COOP_CONE_2(ReferencePoints.blueCoopCone2),
		RED_CONE_3(ReferencePoints.blueCone3),
		RED_CUBE_2(ReferencePoints.blueCube2),
		RED_CONE_4(ReferencePoints.blueCone4);

		public final Pose2d m_scoringPlace; 
		private ScoringOptionBlue(Pose2d scoringPlace) {
			m_scoringPlace = scoringPlace;
		}
	}

}
