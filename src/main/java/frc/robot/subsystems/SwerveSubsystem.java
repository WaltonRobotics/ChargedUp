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

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AutoConstants.*;
import frc.robot.Constants.SwerveK;

public class SwerveSubsystem extends SubsystemBase {
	private final SwerveDriveOdometry odometry;
	private final SwerveDrivePoseEstimator poseEstimator;
	private final SwerveModule[] mSwerveMods;
	private final Pigeon2 gyro = new Pigeon2(Constants.SwerveK.pigeonID, "Canivore");
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			kPThetaController, 0, 0,
			kThetaControllerConstraints);
	private final PIDController xController = new PIDController(kPXController, 0, 0);
	private final PIDController yController = new PIDController(kPYController, 0, 0);

	private final HolonomicDriveController pathController = new HolonomicDriveController(xController, yController, thetaController);

	private final Field2d m_field = new Field2d();

	private ChassisSpeeds m_targetChassisSpeeds;

	public SwerveSubsystem() {
		DashboardManager.addTab(this);
		gyro.configFactoryDefault();
		zeroGyro();

		mSwerveMods = new SwerveModule[] {
			new SwerveModule("Front Left", 0, SwerveK.Mod0.constants),
			new SwerveModule("Front Right", 1, SwerveK.Mod1.constants),
			new SwerveModule("Rear Left", 2, SwerveK.Mod2.constants),
			new SwerveModule("Rear Right", 3, SwerveK.Mod3.constants)
		};

		// 2023 CTRE bugfix
		Timer.delay(.250);
		for (var mod : mSwerveMods) {
			mod.resetToAbsolute();
		}

		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		// thetaController.setTolerance(5, 5);
		odometry = new SwerveDriveOdometry(
			SwerveK.kKinematics,
			getHeading(),
			getModulePositions()
		);

		poseEstimator = new SwerveDrivePoseEstimator(
			SwerveK.kKinematics,
			getHeading(), 
			getModulePositions(),
			getPose()
		);


		m_field.setRobotPose(getPose());
		SmartDashboard.putData(m_field);
	}

	public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace) {
		setModuleStates(SwerveK.kKinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
		
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
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveK.maxSpeed);

		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop);
		}
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(getHeading(), getModulePositions(), pose);
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
	public Rotation2d getHeading() {
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

	public void followAprilTag(double goalDistance, boolean shouldMove) {
		// System.out.println("BUTTON PRESSED");
		var targetOpt = AprilTagHelper.getBestTarget();
		if (targetOpt.isPresent()) {
			// System.out.println("TARGET DETECTED");
			var target = targetOpt.get();
			// Pose3d robotPose3d = new Pose3d(
			// 	getPose().getX(), getPose().getY(), 0, 
			// 	new Rotation3d(0, 0, getPose().getRotation().getDegrees())
			// );

			var tagTr3d = target.getBestCameraToTarget();
			double xMeters = tagTr3d.getX();
			double yMeters = tagTr3d.getY();
			SmartDashboard.putNumber("TagX", xMeters);
			SmartDashboard.putNumber("TagY", yMeters);
			// double zDegrees = Rotation2d.fromRadians(tagTr3d.getRotation().getZ()).getDegrees();
			double zDegrees = target.getYaw();
			SmartDashboard.putNumber("TagYaw", zDegrees);

			// Pose3d tagOffset = robotPose3d.transformBy(target.getBestCameraToTarget());

			// m_field.getObject("bestTag").setPose(tagPose.toPose2d());

			// double xRate = xController.calculate(xMeters, 2);
			// SmartDashboard.putNumber("xEffort", xRate);
			// double yRate = yController.calculate(yMeters, 0.5);
			// SmartDashboard.putNumber("yEffort", yRate);
			double turnRate = thetaController.calculate(zDegrees, 0);
			SmartDashboard.putNumber("thetaEffort", turnRate);

			if (shouldMove) {
				drive(0, 0, turnRate, false, true);
			}
		}
	}

	public ProfiledPIDController getThetaController() {
		return thetaController;
	}

	// public CommandBase getSwerveControllerCommand(Trajectory trajectory) {
	// var resetCommand = new InstantCommand(() ->
	// this.resetOdometry(trajectory.getInitialPose()));
	// var autoSwerveCommand = new SwerveControllerCommand(
	// trajectory,
	// this::getPose,
	// Constants.Swerve.swerveKinematics,
	// new PIDController(Constants.AutoConstants.kPXController, 0, 0),
	// new PIDController(Constants.AutoConstants.kPYController, 0, 0),
	// thetaController,
	// this::setModuleStates,
	// this
	// );
	// return resetCommand.andThen(autoSwerveCommand);
	// }

	@Override
	public void periodic() {
		for (var module : mSwerveMods) {
			module.periodic();
		}
		odometry.update(getHeading(), getModulePositions());
		m_field.setRobotPose(getPose());
		followAprilTag(1, false);
	}
}