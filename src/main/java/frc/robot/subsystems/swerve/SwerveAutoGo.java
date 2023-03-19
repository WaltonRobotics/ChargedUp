package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPointAccessor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAutoGo extends CommandBase {

	private final SwerveSubsystem m_swerve;
	private final List<PathPoint> m_path;
	private final Pose2d m_startPose;
	private final Pose2d m_endPose;

	private PathPlannerTrajectory m_traj1;
	private PathPlannerTrajectory m_traj2;
	private PathPlannerTrajectory m_traj3;

    public SwerveAutoGo(Pose2d startPose, List<PathPoint> path, Pose2d endPose, SwerveSubsystem swerve) {
		m_swerve = swerve;
		m_startPose = startPose; 
		m_path = path;
		m_endPose = endPose;
    }

	@Override
	public void initialize() {
		List<Pose2d> pathPoses = new ArrayList<Pose2d>();
		for (PathPoint point : m_path) {
			pathPoses.add(PathPointAccessor.poseFromPathPointHolo(point));
		}
		
		m_traj1 = SwerveSubsystem.generateTrajectoryToPose(m_startPose, pathPoses.get(0), m_swerve.getFieldRelativeLinearSpeedsMPS());
		m_traj2 = SwerveSubsystem.generateTrajectoryToPose(pathPoses.get(0), pathPoses.get(1), m_swerve.getFieldRelativeLinearSpeedsMPS());
		m_traj3 = SwerveSubsystem.generateTrajectoryToPose(pathPoses.get(1), m_endPose, m_swerve.getFieldRelativeLinearSpeedsMPS());

		var followCmd1 = m_swerve.getFullAuto(m_traj1);
		var followCmd2 = m_swerve.getFullAuto(m_traj2);
		var endCmd = m_swerve.getFullAuto(m_traj3);
		
		followCmd1.andThen(followCmd2).andThen(endCmd).withName("SwerveAutoGoGo").schedule();
	}
}
