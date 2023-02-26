package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.auton.Paths;
import frc.robot.auton.Paths.ReferencePoints;
import frc.robot.vision.AprilTagChooser;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.PathChooser;
import frc.lib.swerve.SwerveDriveState;
import frc.lib.swerve.WaltonPPSwerveControllerCommand;
import frc.lib.swerve.WaltonSwerveAutoBuilder;
import frc.lib.util.DashboardManager;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerUtil;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPointAccessor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.SwerveK.*;
import static frc.robot.Constants.SwerveK.kMaxAngularVelocityRadps;
import static frc.robot.Constants.SwerveK.kMaxVelocityMps;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.lib.vision.EstimatedRobotPose;

public class SwerveSubsystem extends SubsystemBase {
	private final SwerveModule[] m_modules = new SwerveModule[] {
			new SwerveModule("Front Left", 0, Mod0.constants),
			new SwerveModule("Front Right", 1, Mod1.constants),
			new SwerveModule("Rear Left", 2, Mod2.constants),
			new SwerveModule("Rear Right", 3, Mod3.constants)
	};
	private final Pigeon2 m_pigeon = new Pigeon2(Constants.SwerveK.kPigeonCANID, "Canivore");

	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			kPThetaController, 0, 0,
			kThetaControllerConstraints);
	private final PIDController autoThetaController = new PIDController(kPThetaController, 0, kDThetaController);
	private final PIDController xController = new PIDController(kPXController, 0, 0);
	private final PIDController yController = new PIDController(kPYController, 0, 0);
	private final HolonomicDriveController pathController = new HolonomicDriveController(xController, yController,
			thetaController);

	private final Field2d m_field = new Field2d();
	private final SwerveDriveState m_state = new SwerveDriveState(kModuleTranslations);
	private final WaltonSwerveAutoBuilder autoBuilder;

	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
			kKinematics, getHeading(), getModulePositions());

	// private PathPoint currentPathPoint;
	private PathPlannerTrajectory currentTrajectory = new PathPlannerTrajectory();
	private PathPlannerTrajectory trajectoryUsed = new PathPlannerTrajectory();

	private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
			kKinematics,
			getHeading(),
			getModulePositions(),
			new Pose2d());

	private final AprilTagCamera m_apriltagHelper;

	private double m_simYaw = 0;
	
	public SwerveSubsystem(HashMap<String, Command> autoEventMap, AprilTagCamera apriltagHelper) {
		m_apriltagHelper = apriltagHelper;
		DashboardManager.addTab(this);
		m_pigeon.configFactoryDefault();
		zeroGyro();

		Timer.delay(.250);
		for (var mod : m_modules) {
			mod.resetToAbsolute();
		}

		autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		thetaController.setTolerance(Rotation2d.fromDegrees(1).getRadians());
		autoThetaController.setTolerance(Rotation2d.fromDegrees(2.5).getRadians());

		m_state.update(getPose(), getModuleStates(), m_field);
		m_apriltagHelper.updateField2d(m_field);
		DashboardManager.addTabSendable(this, "Field2d", m_field);
		autoBuilder = new WaltonSwerveAutoBuilder(
				this::getPose, // Pose2d supplier
				this::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
				kKinematics, // SwerveDriveKinematics
				kTranslationPID,
				kRotationPID,
				(states) -> setModuleStates(states, false, false), // Module states consumer used to output to the
																	// subsystem
				autoEventMap,
				true,
				this);
	}

	public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace) {
		setModuleStates(kKinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);

	}

	public CommandBase teleopDriveCmd(
			DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation,
			BooleanSupplier robotCentric, BooleanSupplier openLoop) {
		return run(() -> {
			double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.stickDeadband);
			double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.stickDeadband);
			double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.stickDeadband);

			boolean openLoopVal = openLoop.getAsBoolean();
			// if (!openLoopVal) {
			translationVal *= kMaxVelocityMps;
			strafeVal *= kMaxVelocityMps;
			rotationVal *= kMaxAngularVelocityRadps;
			// }

			drive(translationVal, strafeVal, rotationVal, !robotCentric.getAsBoolean(), openLoopVal);
		}).withName("TeleopDrive");
	}

	/**
	 * Basic teleop drive control; ChassisSpeeds values representing vx, vy, and
	 * omega
	 * are converted to individual module states for the robot to follow
	 * 
	 * @param vxMeters     x velocity (forward)
	 * @param vyMeters     y velocity (strafe)
	 * @param omegaRadians angular velocity (rotation CCW+)
	 * @param openLoop     If swerve modules should not use velocity PID
	 */
	public void drive(double vxMeters, double vyMeters, double omegaRadians, boolean fieldRelative, boolean openLoop) {
		ChassisSpeeds targetChassisSpeeds = fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(vxMeters, vyMeters, omegaRadians, getHeading())
				: new ChassisSpeeds(vxMeters, vyMeters, omegaRadians);

		setChassisSpeeds(targetChassisSpeeds, openLoop, false);
	}

	/**
	 * Drive control using angle position (theta) instead of velocity (omega).
	 * The {@link #thetaController theta PID controller} calculates an angular
	 * velocity in order
	 * to reach the target angle, making this method similar to autonomous path
	 * following without
	 * x/y position controllers. This method assumes field-oriented control and is
	 * not affected
	 * by the value of {@link #isFieldRelative}.
	 * 
	 * @param vxMeters       x velocity (forward)
	 * @param vyMeters       y velocity (strafe)
	 * @param targetRotation target angular position
	 * @param openLoop       If swerve modules should not use velocity PID
	 * @return If the drivetrain rotation is within tolerance of the target rotation
	 */
	public boolean drive(double vxMeters, double vyMeters, Rotation2d targetRotation, boolean openLoop) {
		// rotation speed
		double rotationRadians = getPose().getRotation().getRadians();
		double pidOutput = thetaController.calculate(rotationRadians, targetRotation.getRadians());

		// + translation speed
		ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				vxMeters,
				vyMeters,
				pidOutput,
				getHeading());

		setChassisSpeeds(targetChassisSpeeds, openLoop, false);
		return thetaController.atGoal();
	}

	/**
	 * Drive control intended for path following utilizing the
	 * {@link #pathController path controller}.
	 * This method always uses closed-loop control on the modules.
	 * 
	 * @param targetState    Trajectory state containing target translation and
	 *                       velocities
	 * @param targetRotation Target rotation independent of trajectory motion
	 */
	public void drive(Trajectory.State targetState, Rotation2d targetRotation) {
		// determine ChassisSpeeds from path state and positional feedback control from
		// HolonomicDriveController
		ChassisSpeeds targetChassisSpeeds = pathController.calculate(
				getPose(),
				targetState,
				targetRotation);
		// command robot to reach the target ChassisSpeeds
		setChassisSpeeds(targetChassisSpeeds, false, false);
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kKinematics.toChassisSpeeds(getModuleStates());
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop, boolean steerInPlace) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxVelocityMps);

		for (SwerveModule mod : m_modules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop, steerInPlace);
		}
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);

		for (SwerveModule mod : m_modules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false, false);
		}
	}

	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	public void resetEstimatorPose(Pose2d pose) {
		m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : m_modules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : m_modules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public void zeroGyro() {
		m_pigeon.setYaw(0);
	}

	private double getGyroYaw() {
		return m_pigeon.getYaw() - 180;
	}

	// Side to side
	public Rotation2d getHeading() {
		return (kInvertGyro) ? Rotation2d.fromDegrees(360 - getGyroYaw())
				: Rotation2d.fromDegrees(getGyroYaw());
	}

	public void resetPose(Pose2d pose) {
		m_pigeon.setYaw(pose.getRotation().getDegrees());
		resetEstimatorPose(pose); // resets poseEstimator
		resetOdometryPose(pose); // sets odometry to poseEstimator
	}

	/*
	 * Reset wheel odometry pose for autons
	 */
	public void resetOdometryPose(Pose2d pose) {
		m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
	}

	/*
	 * reset wheel odometry to poseEstimator for teleop
	 */
	public void resetOdometryPose() {
		m_odometry.resetPosition(getHeading(), getModulePositions(), m_poseEstimator.getEstimatedPosition());
	}

	/*
	 * Set steer to brake and drive to coast for odometry testing
	 */
	public void testModules() {
		for (var module : m_modules) {
			module.brakeSteerMotor();
			module.coastDriveMotor();
		}
	}

	/*
	 * Set relative drive encoders to 0
	 */
	public void resetDriveEncoders() {
		for (var module : m_modules) {
			module.resetDriveToZero();
		}
	}

	/**
	 * Use gyro to balance on charging station (CURRENTLY UNUSED)
	 */
	public void handleAutoBalance() {
		double xRate = 0;
		double yRate = 0;
		double pitchAngleDegrees = m_pigeon.getPitch();
		double rollAngleDegrees = m_pigeon.getRoll();

		double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
		xRate = Math.sin(pitchAngleRadians) * -1;
		double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
		yRate = Math.sin(rollAngleRadians) * -1;

		drive(xRate, yRate, 0, true, true);

	}

	public void followAprilTag(double yGoal, double xOffset, boolean shouldMove) {
		var targetOpt = m_apriltagHelper.getBestTarget();
		if (targetOpt.isPresent()) {
			System.out.println("TARGET DETECTED");
			var target = targetOpt.get();
			var tagTr3d = target.getBestCameraToTarget();
			double xMeters = tagTr3d.getX();
			double yMeters = tagTr3d.getY();
			double xRate = xController.calculate(xMeters, xOffset);
			double yRate = yController.calculate(yMeters, yGoal);
			double zRadians = target.getYaw();
			System.out.println("ANGLE DIFFERENCE: " + zRadians);
			double turnRate = autoThetaController.calculate(zRadians, 0);
			if (zRadians <= kAlignAngleThresholdRadians) {
				turnRate = 0;
				System.out.println("WITHIN ANGLE TOLERANCE");
			}
			drive(xRate, yRate, turnRate, true, true);
		}
		System.out.println("NO TARGET DETECTED");
	}
	public CommandBase autoScore() {
		// runOnce(curPos = thing; ppt = generate(thing))

		var pathCmd = runOnce(() -> {
			ReferencePoints.currentPoint = PathPoint.fromCurrentHolonomicState(
					getPose(),
					getChassisSpeeds());
			// ReferencePoints.currentPoint = currentPathPoint;
			List<PathPoint> allPoints = new ArrayList<>();
			allPoints.add(ReferencePoints.currentPoint);
			List<PathPoint> chosenPathPoints = PathChooser.GetChosenPath();
			boolean onRed = DriverStation.getAlliance().equals(Alliance.Red);
			double currentX = PathPointAccessor.poseFromPathPointHolo(ReferencePoints.currentPoint).getX();

			for (PathPoint addedPP : chosenPathPoints) {

				double addedX = PathPointAccessor.poseFromPathPointHolo(addedPP).getX();
				if (onRed && currentX < addedX) {
					allPoints.add(addedPP);
				} else if (!onRed && currentX > addedX) {
					allPoints.add(addedPP);
				}
				// else {
				// break;
				// }
			}

			allPoints.add(AprilTagChooser.GetChosenAprilTag());

			currentTrajectory = PathPlanner.generatePath(
					new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared),
					allPoints);
		});

		var followCmd = autoBuilder.followPath(() -> {
			return Optional.ofNullable(currentTrajectory);
		});

		return pathCmd.andThen(followCmd).andThen(goToChosenTag());
	}

	public Paths.ScoringOptions.ScoringOptionRed optimalScoringOptionRed() {
		double roboPoseY = getPose().getY();
		
		if(roboPoseY < Paths.ReferencePoints.ScoringPoints.redCone1.getY()) {
			return Paths.ScoringOptions.ScoringOptionRed.CONE_1;
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.redCube1.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCube1.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCone1.getY())) {
				return Paths.ScoringOptions.ScoringOptionRed.CUBE_1;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionRed.CONE_1;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.redCone2.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCone2.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCube1.getY())) {
				return Paths.ScoringOptions.ScoringOptionRed.CONE_2;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionRed.CUBE_1;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.redCoopCone1.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCoopCone1.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCone2.getY())) {
				return Paths.ScoringOptions.ScoringOptionRed.COOP_CONE_1;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionRed.CONE_2;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.redCoopCube1.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCoopCube1.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCoopCone1.getY())) {
				return Paths.ScoringOptions.ScoringOptionRed.COOP_CUBE_1;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionRed.COOP_CONE_1;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.redCoopCone2.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCoopCone2.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCoopCube1.getY())) {
				return Paths.ScoringOptions.ScoringOptionRed.COOP_CONE_2;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionRed.COOP_CUBE_1;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.redCone3.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCone3.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCoopCone2.getY())) {
				return Paths.ScoringOptions.ScoringOptionRed.CONE_3;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionRed.COOP_CONE_2;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.redCube2.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCube2.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCone3.getY())) {
				return Paths.ScoringOptions.ScoringOptionRed.CUBE_2;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionRed.CONE_3;
			}
		}
		else {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCone4.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.redCube2.getY())) {
				return Paths.ScoringOptions.ScoringOptionRed.CONE_4;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionRed.CUBE_2;
			}
		}
	}

	public Paths.ScoringOptions.ScoringOptionBlue optimalScoringOptionBlue() {
		double roboPoseY = getPose().getY();
		
		if(roboPoseY < Paths.ReferencePoints.ScoringPoints.blueCone1.getY()) {
			return Paths.ScoringOptions.ScoringOptionBlue.CONE_1;
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.blueCube1.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCube1.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCone1.getY())) {
				return Paths.ScoringOptions.ScoringOptionBlue.CUBE_1;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionBlue.CONE_1;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.blueCone2.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCone2.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCube1.getY())) {
				return Paths.ScoringOptions.ScoringOptionBlue.CONE_2;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionBlue.CUBE_1;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.blueCoopCone1.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCoopCone1.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCone2.getY())) {
				return Paths.ScoringOptions.ScoringOptionBlue.COOP_CONE_1;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionBlue.CONE_2;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.blueCoopCube1.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCoopCube1.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCoopCone1.getY())) {
				return Paths.ScoringOptions.ScoringOptionBlue.COOP_CUBE_1;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionBlue.COOP_CONE_1;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.blueCoopCone2.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCoopCone2.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCoopCube1.getY())) {
				return Paths.ScoringOptions.ScoringOptionBlue.COOP_CONE_2;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionBlue.COOP_CUBE_1;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.blueCone3.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCone3.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCoopCone2.getY())) {
				return Paths.ScoringOptions.ScoringOptionBlue.CONE_3;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionBlue.COOP_CONE_2;
			}
		}
		else if(roboPoseY < Paths.ReferencePoints.ScoringPoints.blueCube2.getY()) {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCube2.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCone3.getY())) {
				return Paths.ScoringOptions.ScoringOptionBlue.CUBE_2;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionBlue.CONE_3;
			}
		}
		else {
			if(Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCone4.getY()) < Math.abs(roboPoseY - Paths.ReferencePoints.ScoringPoints.blueCube2.getY())) {
				return Paths.ScoringOptions.ScoringOptionBlue.CONE_4;
			}
			else {
				return Paths.ScoringOptions.ScoringOptionBlue.CUBE_2;
			}
		}
	}

	private CommandBase goToChosenTag() {
		return run(() -> {
			var tagPPPose = PathPointAccessor.poseFromPathPointHolo(AprilTagChooser.GetChosenAprilTag());
			var botPose = getPose();
			double xRate = xController.calculate(botPose.getX(), tagPPPose.getX());
			double yRate = yController.calculate(botPose.getY(), tagPPPose.getY());
			drive(xRate, yRate, new Rotation2d(0), true);
		}).until(() -> xController.atSetpoint() && yController.atSetpoint());
	}

	public CommandBase autoScore(List<PathPoint> path, PathPoint endPt) {
		// runOnce(curPos = thing; ppt = generate(thing))

		var pathCmd = runOnce(() -> {
			ReferencePoints.currentPoint = PathPoint.fromCurrentHolonomicState(
					getPose(),
					getChassisSpeeds());
			// ReferencePoints.currentPoint = currentPathPoint;
			List<PathPoint> allPoints = new ArrayList<>();
			allPoints.add(ReferencePoints.currentPoint);
			List<PathPoint> chosenPathPoints = path;
			boolean onRed = DriverStation.getAlliance().equals(Alliance.Red);
			double currentX = PathPointAccessor.poseFromPathPointHolo(ReferencePoints.currentPoint).getX();

			for (PathPoint addedPP : chosenPathPoints) {

				double addedX = PathPointAccessor.poseFromPathPointHolo(addedPP).getX();
				if (onRed && currentX < addedX) {
					allPoints.add(addedPP);
				} else if (!onRed && currentX > addedX) {
					allPoints.add(addedPP);
				}
				// else {
				// break;
				// }
			}

			allPoints.add(endPt);

			currentTrajectory = PathPlanner.generatePath(
					new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared),
					allPoints);
		});

		var followCmd = run(() -> autoBuilder.followPath(currentTrajectory));

		return pathCmd.andThen(followCmd).andThen(goToChosenPoint(endPt));
	}

	/*
	 * @return Cmd to drive to chosen, pre-specified pathpoint
	 * @endPt The last pathpoint to end up at
	 */
	private CommandBase goToChosenPoint(PathPoint endPt) {
		return run(() -> {
			var tagPPPose = PathPointAccessor.poseFromPathPointHolo(endPt);
			var botPose = getPose();
			double xRate = xController.calculate(botPose.getX(), tagPPPose.getX());
			double yRate = yController.calculate(botPose.getY(), tagPPPose.getY());
			drive(xRate, yRate, new Rotation2d(0), true);
		}).until(() -> xController.atSetpoint() && yController.atSetpoint());
	}

	/*
	 * Create a complete autonomous command group. This will reset the robot pose at
	 * the begininng of the first path, follow paths, trigger events during path following,
	 *  and run commands between paths with stop events
	 */
	public CommandBase getFullAuto(PathPlannerTrajectory trajectory) {
		// resetPose(getPose());
		return autoBuilder.fullAuto(trajectory);
	}

	public CommandBase getFullAuto(List<PathPlannerTrajectory> trajectoryList) {
		return autoBuilder.fullAuto(trajectoryList);
	}

	/*
	 * @return Swerve trajectory cmd for auton pathing that sets
	 * the initial pose and trajectory to the correct alliance color
	 * before running the trajectory
	 * @param trajectory The blue-side original trajectory to run
	 */
	public CommandBase getWaltonPPSwerveAutonCommand(PathPlannerTrajectory trajectory) {
		var resetCmd = runOnce(() -> {
			boolean isRed = DriverStation.getAlliance() == Alliance.Red;
			trajectoryUsed = trajectory;
			PathPlannerTrajectory.PathPlannerState initialState = trajectory.getInitialState();
			if(isRed){
				trajectoryUsed = PathPlannerUtil.transformTrajectoryForAlliance(trajectory, Alliance.Red);
				initialState = PathPlannerUtil.transformStateForAlliance(initialState, Alliance.Red);
			}
			resetPose(initialState.poseMeters);
		});

		Supplier<Optional<PathPlannerTrajectory>> trajSupplier = () -> Optional.of(trajectoryUsed);

		var pathCmd = new WaltonPPSwerveControllerCommand(
			trajSupplier,
			this::getPose,
			kKinematics,
			xController,
			yController,
			autoThetaController,
			this::setModuleStates,
			true,
			this);
		return resetCmd.andThen(pathCmd);
	}

	/*
	 * @return Cmd to rotate to a robot-oriented degrees
	 * @param degrees to rotate to
	 */
	public CommandBase rotateAboutPoint(double degrees) {
		return run(() -> {
			autoThetaController.setSetpoint(Math.toRadians(degrees));
			double thetaEffort = autoThetaController.calculate(getHeading().getRadians());
			if (autoThetaController.getPositionError() > 0.001) {
				thetaEffort += kFThetaController;
			}
			drive(0, 0, thetaEffort, true, true);
		})
				.finallyDo((intr) -> drive(0, 0, 0, false, false))
				.until(() -> autoThetaController.atSetpoint());
	}

	/**
	 * updates odometry & poseEstimator positions
	 * updates field
	 */
	public void updateRobotPose() {
		m_odometry.update(getHeading(), getModulePositions());
		// m_field.getObject("WheelOdo Pos").setPose(m_odometry.getPoseMeters());

		m_poseEstimator.update(getHeading(), getModulePositions());

		// m_apriltagHelper.updateReferencePose(m_odometry.getPoseMeters());
		Optional<EstimatedRobotPose> result = m_apriltagHelper
				.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

		m_state.update(getPose(), getModuleStates(), m_field);

		if (result.isPresent()) {
			EstimatedRobotPose camPose = result.get();
			// updates swervePoseEstimator w/ Apriltag
			m_poseEstimator.addVisionMeasurement(
					camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
			m_field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
		} else {
			// move it way off the screen to make it disappear
			m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
		}
	}

	/*
	 * Autobalance the robot on charge station using gyro
	 */
	public CommandBase autoBalance() {
		return run(() -> { 
			// Uncomment the line below this to simulate the gyroscope axis with a controller joystick
			// Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
			double currentAngle = m_pigeon.getPitch();
		
			double error = 0 - currentAngle;
			double power = -Math.min(Constants.SwerveK.kDriveKP * error, 1);
		
			// // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
			// if (power < 0) {
			//   power *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
			// }
		
			// Limit the max power
			if (Math.abs(power) > 0.4) {
			  power = Math.copySign(0.4, power);
			}
		
			drive(power, 0, new Rotation2d(0), true);
			
			// Debugging Print Statments
			System.out.println("Current Angle: " + currentAngle);
			System.out.println("Error " + error);
			System.out.println("Drive Power: " + power);
		});
	}

	@Override
	public void periodic() {
		for (var module : m_modules) {
			module.periodic();
		}
		updateRobotPose();
	}

	@Override
	public void simulationPeriodic() {
		ChassisSpeeds chassisSpeed = kKinematics.toChassisSpeeds(getModuleStates());
		m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
		m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));

		for (var module : m_modules) {
			module.simulationPeriodic();
		}
	}
}