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
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.SwerveK.*;

import java.util.List;

public class Swerve extends SubsystemBase {
	private final SwerveDriveOdometry swerveOdometry;
	private final SwerveModule[] mSwerveMods;
	private final Pigeon2 gyro = new Pigeon2(Constants.SwerveK.pigeonID, "Canivore");
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			kPThetaController, 0, 0,
			kThetaControllerConstraints);
	private final PIDController xController = new PIDController(kPXController, 0, 0);
	private final PIDController yController = new PIDController(kPYController, 0, 0);
	private final HolonomicDriveController driveController = new HolonomicDriveController(xController, yController, thetaController);

	private final Field2d m_field = new Field2d();

	//TODO: set to neutral, measure encoder tics, find wheel diameter empircally
	public Swerve() {
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

		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		swerveOdometry = new SwerveDriveOdometry(
			Constants.SwerveK.swerveKinematics,
			getYaw(),
			getModulePositions());


		m_field.setRobotPose(getPose());
		DashboardManager.addTabSendable(this, "OdoField", m_field);
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
			fieldRelative ? 
				ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getYaw()) : 
				new ChassisSpeeds(x, y, rotation)
		);
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

	/**
	 * Use Apriltag vision to balance robot
	 * on the charging station
	 */
	public void handleAutoBalance() {
		double xRate = 0;
		double yRate = 0;
		double pitchAngleDegrees = gyro.getPitch();
		double rollAngleDegrees = gyro.getRoll();

		double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
		xRate = Math.sin(pitchAngleRadians) * -1;
		double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
		yRate = Math.sin(rollAngleRadians) * -1;

		drive(xRate, yRate, 0, true, true);
	}

	public CommandBase followAprilTag(double goalDistance, boolean shouldMove) {
		var targetOpt = AprilTagHelper.getBestTarget();
		if (targetOpt.isPresent()) {
			System.out.println("target present");
			var target = targetOpt.get();
			// Pose3d robotPose3d = new Pose3d(
			// 	getPose().getX(), getPose().getY(), 0, 
			// 	new Rotation3d(0, 0, getPose().getRotation().getDegrees())
			// );

            //tag transform 3d
			var tagTr3d = target.getBestCameraToTarget();
			double xMeters = tagTr3d.getX();
			double yMeters = tagTr3d.getY();
			SmartDashboard.putNumber("TagX", xMeters);
			SmartDashboard.putNumber("TagY", yMeters);
			// double zDegrees = Rotation2d.fromRadians(tagTr3d.getRotation().getZ()).getDegrees();
			double zRadians = target.getYaw();
			SmartDashboard.putNumber("TagYaw", zRadians);

			// Pose3d tagOffset = robotPose3d.transformBy(target.getBestCameraToTarget());

			// m_field.getObject("bestTag").setPose(tagPose.toPose2d());

			// double xRate = xController.calculate(xMeters, 2);
			// SmartDashboard.putNumber("xEffort", xRate);
			// double yRate = yController.calculate(yMeters, 0.5);
			// SmartDashboard.putNumber("yEffort", yRate);
			//double turnRate = driveController.calculate(zRadians, 0);
			//SmartDashboard.putNumber("thetaEffort", turnRate);
			// if(zRadians < kAlignAngleThresholdRadians) {
			// 	turnRate = 0;
			// }

			// PathPlannerTrajectory traj = PathPlanner.generatePath(
			// 	new PathConstraints(
			// 		Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
			// 		Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
			// 	new PathPoint(new Translation2d(), Rotation2d.fromDegrees(0)), 
			// 	new PathPoint(new Translation2d(xMeters - 0.5, yMeters), Rotation2d.fromRadians(zRadians))
			// );

			var kinematicsConstraint =
				new SwerveDriveKinematicsConstraint(
					Constants.SwerveK.swerveKinematics,
					Constants.SwerveK.maxSpeed);

			TrajectoryConfig config =
				new TrajectoryConfig(
						Constants.AutoConstants.kMaxSpeedMetersPerSecond,
						Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(Constants.SwerveK.swerveKinematics)
						.addConstraint(kinematicsConstraint);

			Trajectory traj =	
				TrajectoryGenerator.generateTrajectory(
					List.of(getPose(), new Pose2d(new Translation2d(xMeters - 2, yMeters), new Rotation2d(zRadians))),
					config);
			
			return getSwerveControllerCommand(traj);
		}
		return Commands.none();
	}

	public ProfiledPIDController getThetaController() {
		return thetaController;
	}

	public CommandBase getSwerveControllerCommand(Trajectory trajectory) {
		var resetCommand = new InstantCommand(() ->
			this.resetOdometry(trajectory.getInitialPose()));
			var autoSwerveCommand = new SwerveControllerCommand(
				trajectory,
				this::getPose,
				Constants.SwerveK.swerveKinematics,
				driveController,
				this::setModuleStates,
				this
		);
		return resetCommand.andThen(autoSwerveCommand);
	}

	// public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
	// 	return new PPSwerveControllerCommand(
	// 			 traj, 
	// 			 this::getPose, // Pose supplier
	// 			 Constants.SwerveK.swerveKinematics, // SwerveDriveKinematics
	// 			 new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
	// 			 new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
	// 			 new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
	// 			 this::setModuleStates, // Module states consumer
	// 			 false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
	// 			 this // Requires this drive subsystem
	// 			);
	// }

	@Override
	public void periodic() {
		for (var module : mSwerveMods) {
			module.periodic();
		}
		swerveOdometry.update(getYaw(), getModulePositions());
		m_field.setRobotPose(getPose());
	}
}