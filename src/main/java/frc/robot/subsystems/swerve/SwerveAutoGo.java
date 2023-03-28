package frc.robot.subsystems.swerve;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.Paths;

public class SwerveAutoGo extends CommandBase {

	private final SwerveSubsystem m_swerve;
	private final Pose2d[] m_poses;
	private final List<PathPoint> m_path;
	private final Pose2d m_endPose;
	private final Pose2d m_startPose = new Pose2d(new Translation2d(2.32, 4.96), new Rotation2d());

	private PathPlannerTrajectory m_traj;
	private PathPlannerTrajectory m_traj1;
	private PathPlannerTrajectory m_traj2;
	private PathPlannerTrajectory m_traj3;

    public SwerveAutoGo(Pose2d[] poses, List<PathPoint> path, Pose2d endPose, SwerveSubsystem swerve) {
		m_swerve = swerve;
		m_poses = poses;
		m_path = path;
		m_endPose = endPose;
    }

	public SwerveAutoGo(Pose2d endPose, SwerveSubsystem swerve) {
		m_endPose = endPose;
		m_swerve = swerve;
		m_path = null;
		m_poses = null;
	}

	@Override
	public void initialize() {
		m_traj = Paths.generateTrajectoryToPose(m_startPose, m_endPose, m_swerve.getFieldRelativeLinearSpeedsMPS());

		var followCmd = m_swerve.getPPSwerveAutonCmd(m_traj);

		followCmd.withName("SwerveAutoGo").schedule();
		
		// m_traj1 = Paths.generateTrajectoryToPose(m_swerve.getPose(), m_poses[0], m_swerve.getFieldRelativeLinearSpeedsMPS());
		// m_traj2 = PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared),
		// 	m_path);
		// m_traj3 = Paths.generateTrajectoryToPose(m_poses[1], m_endPose, m_swerve.getFieldRelativeLinearSpeedsMPS());

		// var followCmd1 = m_swerve.getPPSwerveAutonCmd(m_traj1);
		// var followCmd2 = m_swerve.getPPSwerveAutonCmd(m_traj2);
		// var endCmd = m_swerve.getPPSwerveAutonCmd(m_traj3);
		
		// followCmd1.andThen(followCmd2).andThen(endCmd).withName("SwerveAutoGoGo").schedule();
	}
}
