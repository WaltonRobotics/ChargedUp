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

public class SwerveAutoGo extends CommandBase {

	private final SwerveSubsystem m_swerve;
	private final PathPoint m_side;
	private final Pose2d m_endPose;

	private PathPlannerTrajectory m_traj;

    public SwerveAutoGo(PathPoint side, Pose2d endPose, SwerveSubsystem swerve) {
		m_swerve = swerve;
		m_side = side;
		m_endPose = endPose;
    }

	public SwerveAutoGo(Pose2d endPose, SwerveSubsystem swerve) {
		m_endPose = endPose;
		m_swerve = swerve;
		m_side = null;
	}

	@Override
	public void initialize() {
		List<PathPoint> path = new ArrayList<>();
		Pose2d currentPose = Flipper.flipIfShould(m_swerve.getPose()); // TODO: figure out how to flip

		PathPoint currentPoint = new PathPoint(currentPose.getTranslation(), Rotation2d.fromDegrees(0), currentPose.getRotation());
	
		path.add(currentPoint);
		path.add(m_side);
		path.add(new PathPoint(m_endPose.getTranslation(), m_endPose.getRotation(), Rotation2d.fromDegrees(-90)));
		
		m_traj = PathPlanner.generatePath(
			new PathConstraints(1, 2),
			path);

		// if (DriverStation.getAlliance().equals(Alliance.Red)) {
		// 	m_traj = Flipper.allianceFlip(m_traj);
		// }

		// PathPlannerTrajectory goToStart = 
		// 	Paths.generateTrajectoryToPose(currentPose, m_traj.getInitialHolonomicPose(), m_swerve.getFieldRelativeLinearSpeedsMPS());

		var followCmd = m_swerve.getPPSwerveAutonCmd(m_traj);

		followCmd.withName("SwerveAutoGo").schedule();
	}
}