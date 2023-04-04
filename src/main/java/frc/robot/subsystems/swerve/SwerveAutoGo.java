// package frc.robot.subsystems.swerve;

// import java.util.ArrayList;
// import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.lib.util.Flipper;
// import frc.robot.auton.Paths;

// public class SwerveAutoGo extends CommandBase {

// 	private final SwerveSubsystem m_swerve;
// 	private final PathPoint m_side;
// 	private final Pose2d m_endPose;

// 	private PathPlannerTrajectory m_traj;

//     public SwerveAutoGo(PathPoint side, Pose2d endPose, SwerveSubsystem swerve) {
// 		m_swerve = swerve;
// 		m_side = side;
// 		m_endPose = endPose;
//     }

// 	public SwerveAutoGo(Pose2d endPose, SwerveSubsystem swerve) {
// 		m_endPose = endPose;
// 		m_swerve = swerve;
// 		m_side = null;
// 	}

// 	@Override
// 	public void initialize() {
// 		List<PathPoint> path = new ArrayList<>();
// 		Pose2d currentPose = Flipper.flipIfShould(m_swerve.getPose());
// 		Pose2d intermediatePose = Flipper.flipIfShould(new Pose2d(currentPose.getX(), m_endPose.getY(), new Rotation2d(0)));
// 		Pose2d endPose = Flipper.flipIfShould(m_endPose);

// 		PathPoint currentPoint = new PathPoint(currentPose.getTranslation(), Flipper.flipIfShould(new Rotation2d(-90)), Flipper.flipIfShould(new Rotation2d(m_swerve.getGyroYaw())));
// 		PathPoint intermediatePoint = new PathPoint(intermediatePose.getTranslation(), Flipper.flipIfShould(new Rotation2d(-135)), Flipper.flipIfShould(intermediatePose.getRotation()));
// 		path.add(currentPoint);
// 		// path.add(m_side);
// 		// path.add(new PathPoint(new Translation2d(currentPose.get, null), null))
// 		path.add(intermediatePoint);
// 		path.add(new PathPoint(m_endPose.getTranslation(),Flipper.flipIfShould(new Rotation2d(-90)), Flipper.flipIfShould(Rotation2d.fromDegrees(0))));
		
// 		m_traj = PathPlanner.generatePath(
// 			new PathConstraints(2, 3.5),
// 			path);

// 		if(Flipper.shouldFlip()) {
// 			m_traj = Flipper.allianceFlip(m_traj);
// 		}

// 		PathPlannerTrajectory goToStart = 
// 			Paths.generateTrajectoryToPose(currentPose, m_traj.getInitialHolonomicPose(), m_swerve.getFieldRelativeLinearSpeedsMPS());

// 		var followCmd1 = m_swerve.goToChosenPoint(intermediatePoint);
// 		var followCmd2 = m_swerve.goToChosenPoint(new PathPoint(m_endPose.getTranslation(), Flipper.flipIfShould(new Rotation2d(-90)), Flipper.flipIfShould(Rotation2d.fromDegrees(0))));

// 		followCmd1.andThen(followCmd2).withName("SwerveAutoGo").withTimeout(1.5).schedule();
// 	}
// }