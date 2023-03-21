package frc.robot.subsystems.swerve;

import java.util.ArrayList;
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
	private List<PathPoint> m_pathUsed = new ArrayList<>();

	private PathPlannerTrajectory m_trajToStart;
	private PathPlannerTrajectory m_trajToEnd;

    public SwerveAutoGo(Pose2d[] poses, List<PathPoint> path, Pose2d endPose, SwerveSubsystem swerve) {
		m_swerve = swerve;
		m_poses = poses;
		m_path = path;
		m_endPose = endPose;
    }

	@Override
	public void initialize() {
		PathPoint current = PathPoint.fromCurrentHolonomicState(m_swerve.getPose(), m_swerve.getChassisSpeeds());
		
		m_pathUsed.add(current);
		m_pathUsed.addAll(m_path);
		
		m_trajToStart = PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared),
			m_pathUsed);
		m_trajToEnd = Paths.generateTrajectoryToPose(m_poses[1], m_endPose, m_swerve.getFieldRelativeLinearSpeedsMPS());

		var followCmd = m_swerve.getFollowPathWithEvents(m_trajToStart);
		var endCmd = m_swerve.getFollowPathWithEvents(m_trajToEnd);
		
		followCmd.andThen(endCmd).withName("SwerveAutoGoGo").schedule();
	}
}
