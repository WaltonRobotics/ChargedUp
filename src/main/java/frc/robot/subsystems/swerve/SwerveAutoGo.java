package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Flipper;
import frc.robot.Constants.AutoConstants;

public class SwerveAutoGo extends CommandBase {

	private final SwerveSubsystem m_swerve;
	private final List<PathPoint> m_path;
	private final Pose2d m_endPose;

	private PathPlannerTrajectory m_traj;

    public SwerveAutoGo(List<PathPoint> path, Pose2d endPose, SwerveSubsystem swerve) {
		m_swerve = swerve;
		m_path = path;
		m_endPose = endPose;
    }

	public SwerveAutoGo(Pose2d endPose, SwerveSubsystem swerve) {
		m_endPose = endPose;
		m_swerve = swerve;
		m_path = null;
	}

	@Override
	public void initialize() {
		List<PathPoint> finalPath = new ArrayList<>();
		Pose2d currentPose = Flipper.flipIfShould(m_swerve.getPose());
		
		finalPath.add(new PathPoint(currentPose.getTranslation(), currentPose.getRotation(), new Rotation2d()));
		finalPath.addAll(m_path);
		finalPath.add(new PathPoint(m_endPose.getTranslation(), m_endPose.getRotation(), Rotation2d.fromDegrees(90)));
		
		m_traj = PathPlanner.generatePath(
			new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared),
			finalPath);

		var followCmd = m_swerve.getPPSwerveAutonCmd(m_traj);

		followCmd.withName("SwerveAutoGo").schedule();
	}
}
