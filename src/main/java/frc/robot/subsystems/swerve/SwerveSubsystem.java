package frc.robot.subsystems.swerve;

import frc.robot.SwerveModule;
import frc.robot.Constants.VisionConstants;
import frc.robot.auton.AutonFactory;
import frc.robot.auton.Paths;
import frc.robot.auton.Paths.PPAutoscoreClass;
import frc.robot.auton.Paths.ReferencePoints;
import frc.robot.vision.AprilTagCamera;
import frc.lib.swerve.SwerveDriveState;
import frc.lib.util.AdvantageScopeUtils;
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
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.vision.EstimatedRobotPose;

public class SwerveSubsystem extends SubsystemBase {
	private final SwerveModule flModule = new SwerveModule("FrontLeft", 0, Mod0.constants);
	private final SwerveModule frModule = new SwerveModule("FrontRight", 1, Mod1.constants);
	private final SwerveModule rlModule = new SwerveModule("RearLeft", 2, Mod2.constants);
	private final SwerveModule rrModule = new SwerveModule("RearRight", 3, Mod3.constants);

	private final SwerveModule[] m_modules = new SwerveModule[] {
		flModule, frModule, rlModule, rrModule
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
	private final Tracer tracer = new Tracer();


	public Timer m_timer = new Timer();

	private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
			kKinematics,
			getHeading(),
			getModulePositions(),
			new Pose2d(),
			new Matrix<>(kOdoStdDevs_DefaultTrust),
			new Matrix<>(kVisionStdDevs_DefaultTrust));

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
		resetToAbsolute();
		tracer.resetTimer();

		autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
		// autoThetaController.setTolerance(Rotation2d.fromDegrees(0.75).getRadians());
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		// thetaController.setTolerance(Rotation2d.fromDegrees(1).getRadians());

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
			getModuleStates()[i] = new SwerveModuleState(
				getModuleStates()[i].speedMetersPerSecond, kModuleTranslations[i].getAngle());
		}
	}

	public void stop() {
		drive(0, 0, Rotation2d.fromDegrees(0), true);
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kKinematics.toChassisSpeeds(getModuleStates());
	}

	public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop, boolean steerInPlace) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxVelocityMps);

		for (SwerveModule mod : m_modules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop, steerInPlace);
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

	public Command chargeStationBatteryFirstC() {
		return Commands.sequence(
			run(()-> drive(2, 0, 0, true, false))
			.until(()->(Math.abs(getGyroPitch()) > 0.13))
			.withTimeout(3),
			Commands.either(
				Commands.sequence(
					run(()-> drive(1.5, 0, 0, true, false)).withTimeout(0.7),
					run(()-> drive(0.7, 0, 0, true, false))
						// .alongWith(Commands.run(()->LightS.getInstance().requestState(States.Climbing)))
						.until(()-> Math.abs(getGyroPitch()) < 0.15),
					run(()-> drive(-0.6, 0, 0, true, false)).withTimeout(1),
					run(()-> drive(0, 0, 0.1, true, false))
						.withTimeout(0.2)), 
				Commands.none(), 
				() -> (Math.abs(getGyroPitch()) > 0.05)
			));
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : m_modules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public void zeroGyro() {
			m_pigeon.setYaw(180);
	}

	public void setYaw(double angle) {
		m_pigeon.setYaw(angle);
	}

	public CommandBase rotate180() {
		return rotateAboutPoint(180);
	}

	public void resetToAbsolute() {
		for (var mod : m_modules) {
			mod.resetToAbsolute();
		}
	}

	protected double getGyroYaw() {
		return m_pigeon.getYaw() ;
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
		var poseDeg = pose.getRotation().getDegrees();
		if(Flipper.shouldFlip() && poseDeg < 180) {
			setYaw(poseDeg - 180);
		} else if(Flipper.shouldFlip() && poseDeg >= 180) {
			setYaw(poseDeg);
		} else if(!Flipper.shouldFlip() && poseDeg < 180) {
			setYaw(poseDeg - 180);
		} else {
			setYaw(poseDeg);
		}
		resetEstimatorPose(pose); // resets poseEstimator
	}

	

	public Translation2d getFieldRelativeLinearSpeedsMPS() {
        ChassisSpeeds robotRelativeSpeeds = kKinematics.toChassisSpeeds(getModuleStates());
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            robotRelativeSpeeds,
            Rotation2d.fromRadians(robotRelativeSpeeds.omegaRadiansPerSecond)
        );
        Translation2d translation = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

        if (translation.getNorm() < 0.01) {
            return new Translation2d();
        } else {
            return translation;
        }
    }

	public CommandBase chasePoseCmd(Supplier<Pose2d> targetSupplier) {
        return new PPChasePoseCommand(
            targetSupplier,
            this::getPose,
            xController,
			yController,
			autoThetaController,
            (chassisSpeeds) -> setChassisSpeeds(chassisSpeeds, false, false),
            (PathPlannerTrajectory traj) -> {},
            (startPose, endPose) -> Paths.generateTrajectoryToPose(startPose, endPose, getFieldRelativeLinearSpeedsMPS()),
            this);
    }

	public void teleOpReset(){
		zeroGyro();
	}

	public CommandBase driveOneDirection(boolean reverse, double speed){
		return run(()-> {
			drive(reverse ? -(speed) : (speed), 0, 0, false, false);
		});
	}

	public CommandBase driveSide(boolean blueLeft){
		return run(()-> {
			double reverse = 1;
			if(DriverStation.getAlliance() == Alliance.Blue){
				reverse = -1;
			}
			drive(0, blueLeft ? -3*reverse : 3*reverse, 0, false, false);
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

	public CommandBase autoScore(Pose2d endPose) {
		return new SwerveAutoGo(ReferencePoints.notBumperPoses, PPAutoscoreClass.notBumpy, endPose, this);
	}

	/**
	 * @return Cmd to drive to chosen, pre-specified pathpoint
	 * 
	 * @endPt The last pathpoint to end up at
	 */
	public CommandBase goToChosenPoint(PathPoint endPose) {
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
		return new DeferredCommand(() -> {
			boolean shouldFlip = Flipper.shouldFlip();
			var newTraj = trajectory;
			if(shouldFlip){
				newTraj = Flipper.allianceFlip(trajectory);
			}
			return autoBuilder.fullAuto(newTraj);
		}, this);
	}

	public CommandBase getTimedFullAuto(PathPlannerTrajectory trajectory) {
		return new DeferredCommand(() -> {
			boolean shouldFlip = Flipper.shouldFlip();
			var newTraj = trajectory;
			if(shouldFlip){
				newTraj = Flipper.allianceFlip(trajectory);
			}
			return autoBuilder.fullAuto(newTraj).withTimeout(trajectory.getTotalTimeSeconds());
		}, this).withTimeout(4);
	}
	// public CommandBase getFullAuto(List<PathPlannerTrajectory> trajectoryList) {
	// 	return autoBuilder.fullAuto(trajectoryList);
	// }

	public CommandBase getPPSwerveAutonCmd(PathPlannerTrajectory trajectory) {
		if (trajectory.getStates().size() == 0) {
			throw new RuntimeException("Empty path given!!!");
		}

		return new DeferredCommand(() -> {
			var newTraj = trajectory;

			if(Flipper.shouldFlip()){
				newTraj = Flipper.allianceFlip(trajectory);
			}

			final var actualNewTraj = newTraj;

			var resetCmd = runOnce(() -> {
				resetPose(actualNewTraj.getInitialHolonomicPose());
			});
			
			var pathCmd = new PPSwerveControllerCommand(
				newTraj,
				this::getPose, // Pose supplier
				kKinematics, // SwerveDriveKinematics
				xController,
				yController,
				autoThetaController,
				(moduleStates) -> setModuleStates(moduleStates, false, false), // Module states consumer
				false, // Should the path be automatically mirrored depending on alliance color.
						// Optional, defaults to true
				this // Requires this drive subsystem
			);
			return resetCmd.andThen(pathCmd);
		}, this);
	}

	public CommandBase getPPSwerveAutonCmd(List<PathPlannerTrajectory> trajList) {
		if (trajList.size() == 0) {
			throw new RuntimeException("Empty path group given!!!");
		}

		return new DeferredCommand(() -> {
			if (trajList.size() == 0) return Commands.print("PPSwerve - Empty path group given!");

			List<PathPlannerTrajectory> newTrajList = new ArrayList<PathPlannerTrajectory>();
			newTrajList.addAll(trajList);


			if(Flipper.shouldFlip()){
				newTrajList.clear();
				for (var traj : trajList) {
					newTrajList.add(Flipper.allianceFlip(traj));
				}
			}

			final var firstTraj = newTrajList.get(0);

			var resetCmd = runOnce(() -> {
				resetPose(firstTraj.getInitialHolonomicPose());
			});

			List<CommandBase> pathCmds = new ArrayList<CommandBase>();

			for (var traj : newTrajList) {
				pathCmds.add(new PPSwerveControllerCommand(
					traj,
					this::getPose, // Pose supplier
					kKinematics, // SwerveDriveKinematics
					xController,
					yController,	
					autoThetaController,
					(moduleStates) -> setModuleStates(moduleStates, false, false), // Module states consumer
					false, // Should the path be automatically mirrored depending on alliance color.
							// Optional, defaults to true
					this // Requires this drive subsystem
				));
			}

			CommandBase[] pathCmdsArr = pathCmds.toArray(new CommandBase[0]);
			return resetCmd.andThen(pathCmdsArr);
		}, this);
	}

	public CommandBase getFollowPathWithEvents(PathPlannerTrajectory traj) {
		return new FollowPathWithEvents(
				getFullAuto(traj),
				traj.getMarkers(),
				AutonFactory.autonEventMap);
	}

	/**
	 * @return Cmd to rotate to a robot-oriented degrees
	 * @param degrees to rotate to
	 */
	public CommandBase rotateAboutPoint(double degrees) {
		return run(() -> {
			autoThetaController.setSetpoint(Math.toRadians(degrees) + getHeading().getRadians());
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
	// public void updateVision() {
	// 	var visionEstBegin = Timer.getFPGATimestamp();
	// 	Optional<EstimatedRobotPose> leftLowPoseOpt = m_apriltagHelper
	// 			.leftLow_getEstPose(m_poseEstimator.getEstimatedPosition());

	// 	Optional<EstimatedRobotPose> rightLowPoseOpt = // Optional.empty();
	// 			m_apriltagHelper.rightLow_getEstPose(m_poseEstimator.getEstimatedPosition());

		

	// 	if (leftLowPoseOpt.isPresent()) {
	// 		var leftLowPose = leftLowPoseOpt.get();
	// 		SmartDashboard.putNumberArray("LeftLowCamPose3d", AdvantageScopeUtils.toDoubleArr(leftLowPose.estimatedPose));

	// 		m_field.getObject("LeftLowCamPose").setPose(leftLowPose.estimatedPose.toPose2d());
	// 		m_poseEstimator.addVisionMeasurement(leftLowPose.estimatedPose.toPose2d(), leftLowPose.timestampSeconds);
	// 	} else {
	// 		m_field.getObject("LeftLowCamPose").setPose(VisionConstants.kWayOutTherePose);
	// 	}

	// 	if (rightLowPoseOpt.isPresent()) {
	// 		var rightLowPose = rightLowPoseOpt.get();
	// 		SmartDashboard.putNumberArray("RightLowCamPose3d", AdvantageScopeUtils.toDoubleArr(rightLowPose.estimatedPose));

	// 		m_field.getObject("RightLowCamPose").setPose(rightLowPose.estimatedPose.toPose2d());
	// 		m_poseEstimator.addVisionMeasurement(rightLowPose.estimatedPose.toPose2d(), rightLowPose.timestampSeconds);
	// 	} else {
	// 		m_field.getObject("RightLowCamPose").setPose(VisionConstants.kWayOutTherePose);
	// 	}

	// 	var visionEstElapsed = Timer.getFPGATimestamp() - visionEstBegin;

		
	// 	SmartDashboard.putNumber("VisionTimeSec", visionEstElapsed);
	// }

	public void updateOdo(){
		var poseEstBegin = Timer.getFPGATimestamp();
		m_poseEstimator.update(getHeading(), getModulePositions());
		m_state.update(getPose(), getModuleStates(), m_field);
		var poseEstElapsed = Timer.getFPGATimestamp() - poseEstBegin;
		SmartDashboard.putNumber("OdoTimeSec", poseEstElapsed);
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
		// tracer.addEpoch(DB_TAB_NAME);
		// if(DriverStation.isAutonomous()){
		// 	updateVision();
		// }

		if(DriverStation.isTeleop()){
			m_apriltagHelper.setDriverMode(true);
		}
		updateOdo();

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