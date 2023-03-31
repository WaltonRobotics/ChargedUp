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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.swerve.PathPointGetters;
import frc.lib.util.Flipper;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.Paths;

public class SwerveAutoGo extends CommandBase {

	private final SwerveSubsystem m_swerve;
	private final List<PathPoint> m_path;
	private final Pose2d m_endPose;

	private PathPlannerTrajectory m_traj;
	private List<PathPoint> m_path2;

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
		List<PathPoint> temp = new ArrayList<>();
		Pose2d currentPose = m_swerve.getPose(); // TODO: figure out how to flip
		
		temp.addAll(m_path);
		temp.add(new PathPoint(m_endPose.getTranslation(), m_endPose.getRotation(), Rotation2d.fromDegrees(-90)));
		
		m_traj = PathPlanner.generatePath(
			new PathConstraints(2, 3),
			temp);


		PathPlannerTrajectory goToStart = 
			Paths.generateTrajectoryToPose(currentPose, m_traj.getInitialHolonomicPose(), m_swerve.getFieldRelativeLinearSpeedsMPS());
		
		if (DriverStation.getAlliance().equals(Alliance.Red)) {
			goToStart = Flipper.allianceFlip(goToStart);
		}

		var followCmd = m_swerve.getPPSwerveAutonCmd(List.of(goToStart, m_traj));

		followCmd.withName("SwerveAutoGo").schedule();
	}
}