package frc.robot.subsystems.swerve;

import frc.robot.SwerveModule;
import frc.robot.auton.AutonFactory;
import frc.robot.vision.AprilTagCamera;
import frc.lib.swerve.SwerveDriveState;
import frc.lib.swerve.WaltonPPSwerveControllerCommand;
import frc.lib.util.DashboardManager;
import frc.lib.util.Flipper;
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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
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

	private final Field2d m_field = new Field2d();
	private final SwerveDriveState m_state = new SwerveDriveState(kModuleTranslations);
	protected final SwerveAutoBuilder autoBuilder;
    protected final LinearFilter m_dropFilter = LinearFilter.highPass(0.3, 0.02);
	protected boolean m_startedBalance = false;


	// private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
	// kKinematics, getHeading(), getModulePositions());

	private PathPlannerTrajectory trajectoryUsed = new PathPlannerTrajectory();

	public Timer m_timer = new Timer();

	private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
			kKinematics,
			getHeading(),
			getModulePositions(),
			new Pose2d());

	private final AprilTagCamera m_apriltagHelper;

	private double m_simYaw = 0;

	private double[] m_xyzDPS = new double[3];

	public SwerveSubsystem(HashMap<String, Command> autoEventMap, AprilTagCamera apriltagHelper) {
		m_apriltagHelper = apriltagHelper;
		DashboardManager.addTab(this);
		m_pigeon.configFactoryDefault();
		m_pigeon.zeroGyroBiasNow();
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
				false,
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

	public double getPitchVelocity() {
		m_pigeon.getRawGyro(m_xyzDPS);

		return m_xyzDPS[1];
	}

	public double getRollVelocity() {
		m_pigeon.getRawGyro(m_xyzDPS);

		return m_xyzDPS[2];
	}

	  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
	public void stopWithX() {
		stop();
		for (int i = 0; i < 4; i++) {
		getModuleStates()[i] =
			new SwerveModuleState(
				getModuleStates()[i].speedMetersPerSecond, kModuleTranslations[i].getAngle());
		}
	}

	public void stop() {
		drive(0, 0, Rotation2d.fromDegrees(0), true);
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

	protected PIDController getThetaController(){
		return autoThetaController;
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

	protected double getGyroYaw() {
		return m_pigeon.getYaw() - 180;
	}

	protected double getGyroRoll() {
		return m_pigeon.getPitch(); // CTRE is Dumb
	}	

	protected double getGyroPitch() {
		return m_pigeon.getRoll(); // CTRE is Dumb
	}
	// Side to side
	public Rotation2d getHeading() {
		return (kInvertGyro) ? Rotation2d.fromDegrees(360 - getGyroYaw())
				: Rotation2d.fromDegrees(getGyroYaw());
	}

	public void resetPose(Pose2d pose) {
		// if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
		// 	zeroGyro();
		// }
		// else{
		// 	setYaw(-180);
		// }
		zeroGyro();
		resetEstimatorPose(pose); // resets poseEstimator
		// resetOdometryPose(pose); // sets odometry to poseEstimator
	}

	public CommandBase driveOneDirection(boolean reverse){
		return run(()-> {
			drive(reverse ? 2 : -2, 0, 0, false, false);
		});
	}

	public CommandBase driveSide(boolean left){
		return run(()-> {
			drive(0, left ? 2 : -2, 0, false, false);
		});
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

	public CommandBase bangBangAutoBalance(){
		double currentYaw = m_pigeon.getYaw();
		double drivePower = 0; 

		if(currentYaw > 135 && currentYaw < 225){
			drivePower *= -1;
		}

		return runOnce(()->{

		});
	}
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
			double xRate = xController.calculate(botPose.getX(),
					PathPointAccessor.poseFromPathPointHolo(endPose).getX());
			double yRate = yController.calculate(botPose.getY(),
					PathPointAccessor.poseFromPathPointHolo(endPose).getY());
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
		return new ProxyCommand(() -> {
			boolean shouldFlip = Flipper.shouldFlip();
			var newTraj = trajectory;
			if(shouldFlip){
				newTraj = Flipper.allianceFlip(trajectory);
			}
			return autoBuilder.fullAuto(newTraj);
		});
	}

	public CommandBase getTimedFullAuto(PathPlannerTrajectory trajectory) {
		return new ProxyCommand(() -> {
			boolean shouldFlip = Flipper.shouldFlip();
			var newTraj = trajectory;
			if(shouldFlip){
				newTraj = Flipper.allianceFlip(trajectory);
			}
			return autoBuilder.fullAuto(newTraj).withTimeout(trajectory.getTotalTimeSeconds());
		}).withTimeout(4);
	}
	// public CommandBase getFullAuto(List<PathPlannerTrajectory> trajectoryList) {
	// 	return autoBuilder.fullAuto(trajectoryList);
	// }

	public CommandBase getPPSwerveAutonCmd(PathPlannerTrajectory trajectory) {
		return new ProxyCommand(() -> {
			boolean shouldFlip = Flipper.shouldFlip();
			var newTraj = trajectory;

			var resetCmd = runOnce(() -> {
				resetPose(trajectory.getInitialHolonomicPose());
			});

			if(shouldFlip){
				resetCmd = runOnce(() -> {
					resetPose(Flipper.allianceFlip(trajectory.getInitialHolonomicPose()));
				});
			}
	
			if(Flipper.shouldFlip()){
				newTraj = Flipper.allianceFlip(trajectory);
			}
			var pathCmd = new PPSwerveControllerCommand(
				newTraj,
				this::getPose, // Pose supplier
				kKinematics, // SwerveDriveKinematics
				xController,
				yController,
				autoThetaController,
				this::setModuleStates, // Module states consumer
				false, // Should the path be automatically mirrored depending on alliance color.
						// Optional, defaults to true
				this // Requires this drive subsystem
			);
			return resetCmd.andThen(pathCmd).withTimeout(2);
		});
	}

	public CommandBase getFollowPathWithEvents(PathPlannerTrajectory traj) {
		return new FollowPathWithEvents(
				getFullAuto(traj),
				traj.getMarkers(),
				AutonFactory.autonEventMap);
	}

	/**
	 * @return Swerve trajectory cmd for auton pathing that sets
	 *         the initial pose and trajectory to the correct alliance color
	 *         before running the trajectory
	 * 
	 * @param trajectory The blue-side original trajectory to run
	 */
	public CommandBase getWaltonPPSwerveAutonCommand(PathPlannerTrajectory trajectory) {
		var resetCmd = runOnce(() -> {
			trajectoryUsed = trajectory;
			PathPlannerState initialState = trajectory.getInitialState();
			if (DriverStation.getAlliance() == Alliance.Red) {
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

	public CommandBase getSwerveAutoCommand(Trajectory traj) {
		return new SwerveControllerCommand(
            traj, 
            this::getPose, 
            kKinematics,
			xController,
	        yController,
            thetaController,
			states -> setModuleStates(states),
			this);
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

		Optional<EstimatedRobotPose> result2 = // Optional.empty();
				m_apriltagHelper.getEstimatedGlobalPose2(m_poseEstimator.getEstimatedPosition());

		m_state.update(getPose(), getModuleStates(), m_field);

		if (result1.isEmpty() && result2.isEmpty()) {
			m_field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
		} else {
			if (result1.isPresent()) {
				camPose1 = result1.get();
				poses.add(camPose1.estimatedPose.toPose2d());
				m_poseEstimator.addVisionMeasurement(camPose1.estimatedPose.toPose2d(), camPose1.timestampSeconds);
			}
			if (result2.isPresent()) {
				camPose2 = result2.get();
				poses.add(camPose2.estimatedPose.toPose2d());
				m_poseEstimator.addVisionMeasurement(camPose2.estimatedPose.toPose2d(), camPose2.timestampSeconds);
			}

			m_field.getObject("Cam Est Pos").setPoses(poses);
		}
	}

	public void lockModules() {
		for (SwerveModule module : m_modules) {
			module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false, true);
		}
	}

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

	public boolean atDistance(Translation2d initialPose, double target) {
		return initialPose.getDistance(getPose().getTranslation()) >= target;
	}

	@Override
	public void periodic() {
		for (var module : m_modules) {
			module.periodic();
		}
		updateRobotPose();

		SmartDashboard.putNumber("Yaw", m_pigeon.getYaw());
		SmartDashboard.putNumber("Pitch", getGyroPitch());
		SmartDashboard.putNumber("Roll", getGyroRoll());

		var filteredPitch = m_dropFilter.calculate(getGyroPitch());
		m_startedBalance = filteredPitch >= -3;

		SmartDashboard.putNumber("HighPassPitch", filteredPitch);
        SmartDashboard.putBoolean("StartedBalance", m_startedBalance);
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