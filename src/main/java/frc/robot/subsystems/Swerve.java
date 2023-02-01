package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.vision.AprilTagHelper;
import frc.lib.util.DashboardManager;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.SwerveK.*;

import java.util.HashMap;

public class Swerve extends SubsystemBase {
	private final SwerveDriveOdometry swerveOdometry;
	private final SwerveModule[] mSwerveMods;
	private final Pigeon2 gyro = new Pigeon2(Constants.SwerveK.pigeonID, "Canivore");
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			kPThetaController, 0, 0,
			kThetaControllerConstraints);
	private final PIDController autoThetaController = new PIDController(kPThetaController, 0, kDThetaController);
	private final PIDController xController = new PIDController(kPXController, 0, 0);
	private final PIDController yController = new PIDController(kPYController, 0, 0);
	private final Field2d m_field = new Field2d();
	private final SwerveAutoBuilder autoBuilder;

	// TODO: set to neutral, measure encoder tics, find wheel diameter empircally
	public Swerve(HashMap<String, Command> autoEventMap) {
		DashboardManager.addTab(this);
		gyro.configFactoryDefault();
		zeroGyro();

		mSwerveMods = new SwerveModule[] {
				new SwerveModule("Front Left", 0, Mod0.constants),
				new SwerveModule("Front Right", 1, Mod1.constants),
				new SwerveModule("Rear Left", 2, Mod2.constants),
				new SwerveModule("Rear Right", 3, Mod3.constants)
		};

		Timer.delay(.250);
		for (var mod : mSwerveMods) {
			mod.resetToAbsolute();
		}

		autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		thetaController.setTolerance(Rotation2d.fromDegrees(1).getRadians());
		autoThetaController.setTolerance(Rotation2d.fromDegrees(2.5).getRadians());
		swerveOdometry = new SwerveDriveOdometry(
				Constants.SwerveK.swerveKinematics,
				getYaw(),
				getModulePositions());

		m_field.setRobotPose(getPose());
		DashboardManager.addTabSendable(this, "OdoField", m_field);
		autoBuilder = new SwerveAutoBuilder(
				this::getPose, // Pose2d supplier
				this::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
				swerveKinematics, // SwerveDriveKinematics
				kTranslationPID,
				kRotationPID,
				this::setModuleStates, // Module states consumer used to output to the drive subsystem
				autoEventMap,
				true,
				this);
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		SwerveModuleState[] swerveModuleStates = Constants.SwerveK.swerveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
						translation.getX(),
						translation.getY(),
						rotation,
						getYaw())
						: new ChassisSpeeds(
								translation.getX(),
								translation.getY(),
								rotation));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveK.maxSpeed);

		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	public void drive(double x, double y, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getYaw())
						: new ChassisSpeeds(x, y, rotation));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);

		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	public Pose2d getPose() {
		return swerveOdometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		zeroGyro();
		swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : mSwerveMods) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : mSwerveMods) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public void zeroGyro() {
		gyro.setYaw(0);
	}

	// side to side
	public Rotation2d getYaw() {
		return (Constants.SwerveK.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
				: Rotation2d.fromDegrees(gyro.getYaw());
	}

	// TODO:may need to reset pose estimator when april tags work
	public void resetPose(Pose2d pose) {
		resetOdometry(pose);
	}

	/**
	 * Use Apriltag vision to balance robot
	 * on the charging station
	 */
	public void handleAutoBalance() {
		double xRate = 0;
		double yRate = 0;
		
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

	public CommandBase rotateAboutPoint(double degrees) {
		return run(() -> {
			autoThetaController.setSetpoint(Math.toRadians(degrees));
			double thetaEffort = autoThetaController.calculate(getYaw().getRadians());
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
		for (var module : mSwerveMods) {
			module.periodic();
		}
		swerveOdometry.update(getYaw(), getModulePositions());
		m_field.setRobotPose(getPose());
	}
}