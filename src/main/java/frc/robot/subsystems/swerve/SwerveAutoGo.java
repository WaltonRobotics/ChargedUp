package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPointAccessor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.Paths.ReferencePoints;

public class SwerveAutoGo extends CommandBase {

	private final SwerveSubsystem m_swerve;
	private final List<PathPoint> m_path;

	private PathPlannerTrajectory m_traj;


	public static CommandBase create(List<PathPoint> path, Pose2d endPose, SwerveSubsystem swerve) {
		var autoGo = new SwerveAutoGo(path, swerve);
		var followCmd = swerve.autoBuilder.followPath(autoGo.m_traj);	
		return autoGo.andThen(followCmd).andThen(swerve.goToChosenPoint(endPose));
	}

    private SwerveAutoGo(List<PathPoint> path, SwerveSubsystem swerve) {
		m_swerve = swerve;
		m_path = path;
    }

	@Override
	public void initialize() {
		ReferencePoints.currentPoint = PathPoint.fromCurrentHolonomicState(
			m_swerve.getPose(),
			m_swerve.getChassisSpeeds());
		List<PathPoint> allPoints = new ArrayList<>();
		allPoints.add(ReferencePoints.currentPoint);
		boolean onRed = DriverStation.getAlliance().equals(Alliance.Red);
		double currentX = PathPointAccessor.poseFromPathPointHolo(ReferencePoints.currentPoint).getX();

		for (PathPoint addedPP : m_path) {
			double addedX = PathPointAccessor.poseFromPathPointHolo(addedPP).getX();
			if (onRed && currentX < addedX) {
				allPoints.add(addedPP);
			} else if (!onRed && currentX > addedX) {
				allPoints.add(addedPP);
			}
		}
		
		m_traj = PathPlanner.generatePath(
			new PathConstraints(
				AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared),
			allPoints
		);
	}
}
