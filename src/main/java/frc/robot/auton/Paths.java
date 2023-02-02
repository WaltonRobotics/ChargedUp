package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;

import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;

import java.util.List;

public class Paths {
	// this is not needed for PPSwerve autons
	public static final TrajectoryConfig config = 
		new TrajectoryConfig(
			kMaxSpeedMetersPerSecond,
			kMaxAccelerationMetersPerSecondSquared
		).setKinematics(Constants.SwerveK.kKinematics);

	public static final PathConstraints kPPConstraints = 
		new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);


	public static final class PPPaths {
		public static final PathPlannerTrajectory oneMeter = 
			PathPlanner.loadPath("oneMeter", kPPConstraints);

			
		public static final List<PathPlannerTrajectory> twoPiece = 
			PathPlanner.loadPathGroup("banks3", false, kPPConstraints);

		public static final PathPlannerTrajectory threePiece2 = 
			PathPlanner.loadPath("threePiece2", kPPConstraints);
	}
}
