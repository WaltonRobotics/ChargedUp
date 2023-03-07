package frc.robot.subsystems.swerve;

import frc.robot.SwerveModule;
import frc.robot.auton.AutonFactory;
import frc.robot.auton.Paths.ReferencePoints;
import frc.robot.auton.Paths.ReferencePoints.ScoringPoints;
import frc.robot.vision.AprilTagCamera;
import frc.lib.swerve.SwerveDriveState;
import frc.lib.swerve.WaltonPPSwerveControllerCommand;
import frc.lib.util.DashboardManager;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.SwerveK.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.*;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.EstimatedRobotPose;
import frc.robot.Constants.AutoConstants;

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

	private final Field2d m_field = new Field2d();
	private final SwerveDriveState m_state = new SwerveDriveState(kModuleTranslations);
	protected final SwerveAutoBuilder autoBuilder;

	// private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
	// 		kKinematics, getHeading(), getModulePositions());


	// private PathPoint currentPathPoint;
	private PathPlannerTrajectory currentTrajectory = new PathPlannerTrajectory();
	private PathPlannerTrajectory trajectoryUsed = new PathPlannerTrajectory();

	public Timer m_timer = new Timer();


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
		// zeroGyro();

		Timer.delay(.250);
		resetModsToAbs();

		autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
		autoThetaController.setTolerance(Rotation2d.fromDegrees(0.75).getRadians());
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		thetaController.setTolerance(Rotation2d.fromDegrees(1).getRadians());

		m_state.update(getPose(), getModuleStates(), m_field);
		m_apriltagHelper.updateField2d(m_field);
		DashboardManager.addTabSendable(this, "Field2d", m_field);

		autoBuilder = new SwerveAutoBuilder(
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

		DashboardManager.addTabSendable(this, "XCtrl", xController);
		DashboardManager.addTabSendable(this, "YCtrl", yController);
		DashboardManager.addTabSendable(this, "AutoThetaCtrl", autoThetaController);
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
			double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.stickDeadband) * .80;

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

	public void setYaw(double angle) {
		m_pigeon.setYaw(angle);
	}

	public CommandBase rotate180() {
		return rotateAboutPoint(180);
	}

	public void resetModsToAbs() {
		for (var mod : m_modules) {
			mod.resetToAbsolute();
		}
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
		// resetOdometryPose(pose); // sets odometry to poseEstimator
	}

	// /*
	//  * Reset wheel odometry pose for autons
	//  */
	// public void resetOdometryPose(Pose2d pose) {
	// 	m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
	// }

	// /*
	//  * reset wheel odometry to poseEstimator for teleop
	//  */
	// public void resetOdometryPose() {
	// 	m_odometry.resetPosition(getHeading(), getModulePositions(), m_poseEstimator.getEstimatedPosition());
	// }

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

	// public void followAprilTag(double yGoal, double xOffset, boolean shouldMove) {
	// 	var targetOpt = m_apriltagHelper.getBestTarget1(); // TODO: check to see if we need to use both cameras for this
	// 	if (targetOpt.isPresent()) {
	// 		System.out.println("TARGET DETECTED");
	// 		var target = targetOpt.get();
	// 		var tagTr3d = target.getBestCameraToTarget();
	// 		double xMeters = tagTr3d.getX();
	// 		double yMeters = tagTr3d.getY();
	// 		double xRate = xController.calculate(xMeters, xOffset);
	// 		double yRate = yController.calculate(yMeters, yGoal);
	// 		double zRadians = target.getYaw();
	// 		System.out.println("ANGLE DIFFERENCE: " + zRadians);
	// 		double turnRate = autoThetaController.calculate(zRadians, 0);
	// 		if (zRadians <= kAlignAngleThresholdRadians) {
	// 			turnRate = 0;
	// 			System.out.println("WITHIN ANGLE TOLERANCE");
	// 		}
	// 		drive(xRate, yRate, turnRate, true, true);
	// 	}
	// 	System.out.println("NO TARGET DETECTED");
	// }

	// public CommandBase autoScore() {
	// 	// runOnce(curPos = thing; ppt = generate(thing))

	// 	var pathCmd = runOnce(() -> {
	// 		ReferencePoints.currentPoint = PathPoint.fromCurrentHolonomicState(
	// 				getPose(),
	// 				getChassisSpeeds());
	// 		// ReferencePoints.currentPoint = currentPathPoint;
	// 		List<PathPoint> allPoints = new ArrayList<>();
	// 		allPoints.add(ReferencePoints.currentPoint);
	// 		List<PathPoint> chosenPathPoints = PathChooser.GetChosenPath();
	// 		boolean onRed = DriverStation.getAlliance().equals(Alliance.Red);
	// 		double currentX = PathPointAccessor.poseFromPathPointHolo(ReferencePoints.currentPoint).getX();

	// 		for (PathPoint addedPP : chosenPathPoints) {

	// 			double addedX = PathPointAccessor.poseFromPathPointHolo(addedPP).getX();
	// 			if (onRed && currentX < addedX) {
	// 				allPoints.add(addedPP);
	// 			} else if (!onRed && currentX > addedX) {
	// 				allPoints.add(addedPP);
	// 			}
	// 			// else {
	// 			// break;
	// 			// }
	// 		}

	// 		allPoints.add(AprilTagChooser.GetChosenAprilTag());

	// 		currentTrajectory = PathPlanner.generatePath(
	// 				new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared),
	// 				allPoints);
	// 	});

	// 	var followCmd = autoBuilder.followPath(() -> {
	// 		return Optional.ofNullable(currentTrajectory);
	// 	});

	// 	return pathCmd.andThen(followCmd).andThen(goToChosenTag());
	// }

	// private CommandBase goToChosenTag() {
	// 	return run(() -> {
	// 		var tagPPPose = PathPointAccessor.poseFromPathPointHolo(AprilTagChooser.GetChosenAprilTag());
	// 		var botPose = getPose();
	// 		double xRate = xController.calculate(botPose.getX(), tagPPPose.getX());
	// 		double yRate = yController.calculate(botPose.getY(), tagPPPose.getY());
	// 		drive(xRate, yRate, new Rotation2d(0), true);
	// 	}).until(() -> xController.atSetpoint() && yController.atSetpoint());
	// }

	public CommandBase autoScore(List<PathPoint> path, PathPoint endPose) {
		return new SwerveAutoGo(path, endPose, this);
	}

	/**
	 * @return Cmd to drive to chosen, pre-specified pathpoint
	 * 
	 * @endPt The last pathpoint to end up at
	 */
	protected CommandBase goToChosenPoint(PathPoint endPose) {
		return run(() -> {
			var botPose = getPose();
			double xRate = xController.calculate(botPose.getX(), PathPointAccessor.poseFromPathPointHolo(endPose).getX());
			double yRate = yController.calculate(botPose.getY(), PathPointAccessor.poseFromPathPointHolo(endPose).getY());
			drive(xRate, yRate, new Rotation2d(0), true);
		}).until(() -> xController.atSetpoint() && yController.atSetpoint());
	}

	/*
	 * Create a complete autonomous command group. This will reset the robot pose at
	 * the begininng of the first path, follow paths, trigger events during path
	 * following,
	 * and run commands between paths with stop events
	 */
	public CommandBase getFullAuto(PathPlannerTrajectory trajectory) {
		return autoBuilder.fullAuto(trajectory);
	}

	public CommandBase getFullAuto(List<PathPlannerTrajectory> trajectoryList) {
		return autoBuilder.fullAuto(trajectoryList);
	}

	public CommandBase getPPSwerveAutonCmd(PathPlannerTrajectory trajectory){
		var resetCmd = runOnce(()->{
			resetPose(trajectory.getInitialHolonomicPose());
		});
		var pathCmd = new PPSwerveControllerCommand(
            trajectory, 
            this::getPose, // Pose supplier
            kKinematics, // SwerveDriveKinematics
            xController,
			yController,
			autoThetaController,
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        );
		return resetCmd.andThen(pathCmd);

	}

	public CommandBase getFollowPathWithEvents(PathPlannerTrajectory traj) {
		return new FollowPathWithEvents(
					getFullAuto(traj),
					traj.getMarkers(),
					AutonFactory.autonEventMap
		);
	}

	/**
	 * @return Swerve trajectory cmd for auton pathing that sets
	 * the initial pose and trajectory to the correct alliance color
	 * before running the trajectory
	 * 
	 * @param trajectory The blue-side original trajectory to run
	 */
	public CommandBase getWaltonPPSwerveAutonCommand(PathPlannerTrajectory trajectory) {
		var resetCmd = runOnce(() -> {
			trajectoryUsed = trajectory;
			PathPlannerState initialState = trajectory.getInitialState();
			if(DriverStation.getAlliance() == Alliance.Red){
				initialState = ReflectedTransform.reflectiveTransformState(trajectory.getInitialState());
			}
			Pose2d initialPose = initialState.poseMeters;
			resetPose(initialPose);
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
	/**
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
		EstimatedRobotPose camPose1;
		EstimatedRobotPose camPose2;

		List<Pose2d> poses = new ArrayList<>();

		// m_odometry.update(getHeading(), getModulePositions());
		// m_field.getObject("WheelOdo Pos").setPose(m_odometry.getPoseMeters());

		m_poseEstimator.update(getHeading(), getModulePositions());

		Optional<EstimatedRobotPose> result1 = m_apriltagHelper
				.getEstimatedGlobalPose1(m_poseEstimator.getEstimatedPosition());
		
		Optional<EstimatedRobotPose> result2 = //Optional.empty();
		m_apriltagHelper.getEstimatedGlobalPose2(m_poseEstimator.getEstimatedPosition());

		m_state.update(getPose(), getModuleStates(), m_field);

		if (result1.isEmpty() && result2.isEmpty()) {
			m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
		} else {
			if(result1.isPresent()) {
				camPose1 = result1.get();
				poses.add(camPose1.estimatedPose.toPose2d());
				m_poseEstimator.addVisionMeasurement(camPose1.estimatedPose.toPose2d(), camPose1.timestampSeconds);
			}
			if(result2.isPresent()) {
				camPose2 = result2.get();
				poses.add(camPose2.estimatedPose.toPose2d());
				m_poseEstimator.addVisionMeasurement(camPose2.estimatedPose.toPose2d(), camPose2.timestampSeconds);
			}

			m_field.getObject("Cam Est Pos").setPoses(poses);
		}
	}

	/*
	 * Autobalance the robot on charge station using gyro
	 */
	public CommandBase autoBalance() {
		return run(() -> {
			// Uncomment the line below this to simulate the gyroscope axis with a
			// controller joystick
			// Double currentAngle = -1 *
			// Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
			double currentAngle = m_pigeon.getPitch();

			double error = 0 - currentAngle;
			double power = -Math.min(Constants.SwerveK.kDriveKP * error, 1);

			// // Our robot needed an extra push to drive up in reverse, probably due to
			// weight imbalances
			// if (power < 0) {
			// power *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
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

	// public CommandBase alignToScoreCubeCmd() {
	// 	return goToChosenPoint(alignToScoreCube());
	// }

	// private Pose2d alignToScoreCube() {
	// 	double yValue = getPose().getY();
	// 	Pose2d closest;
	// 	Pose2d finalDestination;

	// 	closest = ScoringPoints.scoringPoints[0];
	// 	for (int i = 1; i <= 8; i++) {
	// 		if (Math.abs(yValue - closest.getTranslation().getY()) > Math
	// 				.abs(yValue - ScoringPoints.scoringPoints[i].getTranslation().getY())) {
	// 			closest = ScoringPoints.scoringPoints[i];
	// 		}
	// 	}
	// 	finalDestination = new Pose2d(closest.getTranslation(), closest.getRotation());
		
	// 	return finalDestination;
	// }

	public void autoReset() {
		if (m_timer.get() > 10) {
			m_timer.restart();
		} else {
			if (getChassisSpeeds().vxMetersPerSecond == 0 && getChassisSpeeds().vyMetersPerSecond == 0) {
				Timer.delay(1);
				if (getChassisSpeeds().vxMetersPerSecond == 0 && getChassisSpeeds().vyMetersPerSecond == 0) {
					for (SwerveModule module : m_modules) {
						module.resetToAbsolute();
					}
				}
			}
		}
	}

	// public enum AutoScoreState {
	// 	RED_CONE_1(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCone1),
	// 	RED_CUBE_2(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCube2),
	// 	RED_CONE_3(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCone3),
	// 	RED_COOP_CONE_4(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCoopCone4),
	// 	RED_COOP_CUBE_5(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCoopCube5),
	// 	RED_COOP_CONE_6(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCoopCone6),
	// 	RED_CONE_7(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCone7),
	// 	RED_CUBE_8(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCube8),
	// 	RED_CONE_9(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCone9);

	// 	public final List<PathPoint> path;
	// 	public final Pose2d endPose;
		
	// 	private AutoScoreState(List<PathPoint> path, Pose2d endPose) {
	// 		this.path = path;
	// 		this.endPose = endPose;
	// 	}
	// }

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