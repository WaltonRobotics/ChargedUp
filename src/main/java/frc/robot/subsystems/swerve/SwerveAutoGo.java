package frc.robot.subsystems.swerve;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPointAccessor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.swerve.PathPointGetters;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.Paths;

public class SwerveAutoGo extends CommandBase {

	private final SwerveSubsystem m_swerve;
	private final Pose2d[] m_poses;
	private final List<PathPoint> m_path;
	private final Pose2d m_endPose;

	// private PathPlannerTrajectory m_traj1;
	private PathPlannerTrajectory m_traj1;
	private PathPlannerTrajectory m_traj2;

    public SwerveAutoGo(Pose2d[] poses, List<PathPoint> path, Pose2d endPose, SwerveSubsystem swerve) {
		m_swerve = swerve;
		m_poses = poses;
		m_path = path;
		m_endPose = endPose;
    }

	@Override
	public void initialize() {
		// idek bruh
		m_path.add(0, new PathPoint(m_swerve.getPose().getTranslation(), new Rotation2d(), m_swerve.getPose().getRotation()));
		
		m_traj1 = PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, 
			AutoConstants.kMaxAccelerationMetersPerSecondSquared), m_path);
		m_traj2 = Paths.generateTrajectoryToPose(m_poses[1], m_endPose, m_swerve.getFieldRelativeLinearSpeedsMPS()); 
		
		var followCmd = m_swerve.getFollowPathWithEvents(m_traj1);
		var endCmd = m_swerve.getFollowPathWithEvents(m_traj2);
		
		followCmd.andThen(endCmd).withName("SwerveAutoGoGo").schedule();
	}
}
