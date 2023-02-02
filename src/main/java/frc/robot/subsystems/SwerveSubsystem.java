package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.vision.AprilTagHelper;
import frc.lib.swerve.SwerveDriveState;
import frc.lib.util.DashboardManager;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveK;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.SwerveK.*;



public class SwerveSubsystem extends SubsystemBase {
	// private final SwerveDriveOdometry odometry; // PoseEstimator used instead
	private final Pigeon2 m_pigeon = new Pigeon2(Constants.SwerveK.kPigeonCANID, "Canivore");

	private final SwerveModule[] m_modules = new SwerveModule[] {
		new SwerveModule("Front Left", 0, Mod0.constants),
		new SwerveModule("Front Right", 1, Mod1.constants),
		new SwerveModule("Rear Left", 2, Mod2.constants),
		new SwerveModule("Rear Right", 3, Mod3.constants)
	};

	private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
		kKinematics,
		getHeading(),
		getModulePositions(),
		new Pose2d()
	);

	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			kPThetaController, 0, 0,
			kThetaControllerConstraints);
	private final PIDController autoThetaController = new PIDController(kPThetaController, 0, kDThetaController);
	private final PIDController xController = new PIDController(kPXController, 0, 0);
	private final PIDController yController = new PIDController(kPYController, 0, 0);

	private final HolonomicDriveController pathController = new HolonomicDriveController(xController, yController, thetaController);

	private final Field2d m_field = new Field2d();
	private final SwerveDriveState m_swerveState = new SwerveDriveState(kModuleTranslations);
	private final SwerveAutoBuilder autoBuilder;

	private double m_simYaw;

	// TODO: set to neutral, measure encoder tics, find wheel diameter empircally
	public SwerveSubsystem(HashMap<String, Command> autoEventMap) {
		DashboardManager.addTab(this);
		m_pigeon.configFactoryDefault();
		zeroGyro();

		// 2023 CTRE bugfix
		Timer.delay(.250);
		for (var mod : m_modules) {
			mod.resetToAbsolute();
		}

		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		thetaController.setTolerance(Rotation2d.fromDegrees(1).getRadians());
		autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
		autoThetaController.setTolerance(Rotation2d.fromDegrees(1).getRadians());


		m_swerveState.update(getPose(), getModuleStates(), m_field);
		DashboardManager.addTabSendable(this, "Field2d", m_field);
		autoBuilder = new SwerveAutoBuilder(
				this::getPose, // Pose2d supplier
				this::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
				kKinematics, // SwerveDriveKinematics
				kTranslationPID,
				kRotationPID,
				(states) -> setModuleStates(states, false, false), // Module states consumer used to output to the drive subsystem
				autoEventMap,
				true,
				this);
	}

	public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace) {
		setModuleStates(SwerveK.kKinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
		
	}

	public CommandBase teleopDriveCmd(
		DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation,
		BooleanSupplier robotCentric, BooleanSupplier openLoop
	) {
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
	 * Basic teleop drive control; ChassisSpeeds values representing vx, vy, and omega
	 * are converted to individual module states for the robot to follow
	 * @param vxMeters x velocity (forward)
	 * @param vyMeters y velocity (strafe)
	 * @param omegaRadians angular velocity (rotation CCW+)
	 * @param openLoop If swerve modules should not use velocity PID
	 */
	public void drive(double vxMeters, double vyMeters, double omegaRadians, boolean fieldRelative, boolean openLoop) {
		ChassisSpeeds targetChassisSpeeds = fieldRelative ? 
			ChassisSpeeds.fromFieldRelativeSpeeds(vxMeters, vyMeters, omegaRadians, getHeading()) : 
			new ChassisSpeeds(vxMeters, vyMeters, omegaRadians);

		setChassisSpeeds(targetChassisSpeeds, openLoop, false);
	}

	 /**
     * Drive control using angle position (theta) instead of velocity (omega).
     * The {@link #thetaController theta PID controller} calculates an angular velocity in order
     * to reach the target angle, making this method similar to autonomous path following without
     * x/y position controllers. This method assumes field-oriented control and is not affected
     * by the value of {@link #isFieldRelative}.
     * @param vxMeters x velocity (forward)
     * @param vyMeters y velocity (strafe)
     * @param targetRotation target angular position
     * @param openLoop If swerve modules should not use velocity PID
     * @return If the drivetrain rotation is within tolerance of the target rotation
     */
    public boolean drive(double vxMeters, double vyMeters, Rotation2d targetRotation, boolean openLoop){
			// rotation speed
			double rotationRadians = getPose().getRotation().getRadians();
			double pidOutput = thetaController.calculate(rotationRadians, targetRotation.getRadians());

			// + translation speed
			ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
					vxMeters,
					vyMeters,
					pidOutput,
					getHeading()
			);

			setChassisSpeeds(targetChassisSpeeds, openLoop, false);
			return thetaController.atGoal();
	}

	/**
     * Drive control intended for path following utilizing the {@link #pathController path controller}.
     * This method always uses closed-loop control on the modules.
     * @param targetState Trajectory state containing target translation and velocities
     * @param targetRotation Target rotation independent of trajectory motion
     */
    public void drive(Trajectory.State targetState, Rotation2d targetRotation){
			// determine ChassisSpeeds from path state and positional feedback control from HolonomicDriveController
			ChassisSpeeds targetChassisSpeeds = pathController.calculate(
					getPose(),
					targetState,
					targetRotation
			);
			// command robot to reach the target ChassisSpeeds
			setChassisSpeeds(targetChassisSpeeds, false, false);
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop, boolean steerInPlace) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveK.kMaxVelocityMps);

		for (SwerveModule mod : m_modules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop, steerInPlace);
		}
	}

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {
		poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
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

	// side to side
	public Rotation2d getHeading() {
		return (Constants.SwerveK.kInvertGyro) ? Rotation2d.fromDegrees(360 - m_pigeon.getYaw())
				: Rotation2d.fromDegrees(m_pigeon.getYaw());
	}

	// TODO:may need to reset pose estimator when april tags work
	public void resetPose(Pose2d pose) {
		m_pigeon.setYaw(pose.getRotation().getDegrees());
		resetOdometry(pose);
	}

	/**
	 * Use Apriltag vision to balance robot
	 * on the charging station
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
		var targetOpt = AprilTagHelper.getBestTarget();
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

	/*
	 * Create a complete autonomous command group. This will reset the robot pose at
	 * the begininng of
	 * the first path, follow paths, trigger events during path following, and run
	 * commands between
	 * paths with stop events
	 */
	public CommandBase getFullAuto(PathPlannerTrajectory trajectory) {
		return autoBuilder.fullAuto(trajectory);
	}
	
	public CommandBase getFullAuto(List<PathPlannerTrajectory> trajectoryList) {
		return autoBuilder.fullAuto(trajectoryList);
	}

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

	@Override
	public void periodic() {
		for (var module : m_modules) {
			module.periodic();
		}
		// odometry.update(getHeading(), getModulePositions());
		poseEstimator.update(getHeading(), getModulePositions());

		m_swerveState.update(getPose(), getModuleStates(), m_field);
	}

	@Override
	public void simulationPeriodic() {
		ChassisSpeeds chassisSpeed = kKinematics.toChassisSpeeds(getModuleStates());
		m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
		m_pigeon.getSimCollection().setRawHeading(Units.radiansToDegrees(m_simYaw));

		for (var module : m_modules) {
			module.simulationPeriodic();
		}
	}
}