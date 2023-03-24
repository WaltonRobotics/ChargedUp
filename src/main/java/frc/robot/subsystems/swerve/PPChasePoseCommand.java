package frc.robot.subsystems.swerve;

import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link PathPlannerTrajectory}
 * with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from
 * those and used in
 * velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class PPChasePoseCommand extends CommandBase implements Loggable {
    @Log(methodName = "get")
    private final Timer m_timer = new Timer();
    private Supplier<Pose2d> m_targetPose;
    private PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;
    private final Consumer<ChassisSpeeds> m_outputChassisSpeedsRobotRelative;
    private final Consumer<PathPlannerTrajectory> m_outputTrajectory;
    private final BiFunction<Pose2d, Pose2d, PathPlannerTrajectory> m_trajectoryGenerator;
    private Pose2d m_lastRegenTarget;
    private SwerveSubsystem m_swerve;

    /**
     * Constructs a command to follow a moving target pose. Uses PathPlanner trajectories when the target is more than 0.2 m away.
     * @param targetPose A Supplier for the target pose.
     * @param pose A Supplier for the current pose.
     * @param driveController The PPHolonomicDriveController to use.
     * @param outputChassisSpeedsFieldRelative
     * @param trajectoryDebugOutput
     * @param trajectoryGenerator
     * @param swerve
     */
    public PPChasePoseCommand(
        Supplier<Pose2d> targetPose,
        Supplier<Pose2d> pose,
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        Consumer<ChassisSpeeds> outputChassisSpeedsFieldRelative,
        Consumer<PathPlannerTrajectory> trajectoryDebugOutput,
        BiFunction<Pose2d, Pose2d, PathPlannerTrajectory> trajectoryGenerator,
        SwerveSubsystem swerve) {
        
        m_targetPose = targetPose;
        m_pose = pose;
        m_lastRegenTarget = m_targetPose.get();

        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;

        m_outputTrajectory = trajectoryDebugOutput;
        m_trajectoryGenerator = trajectoryGenerator;
        m_outputChassisSpeedsRobotRelative = outputChassisSpeedsFieldRelative;
        m_swerve = swerve;
        addRequirements(m_swerve);
        }


    @Override
    public void initialize() {
        regenTrajectory();
    }

    private void regenTrajectory() {
        m_timer.reset();
        m_timer.start();
        m_lastRegenTarget  = m_targetPose.get();
        m_trajectory = m_trajectoryGenerator.apply(m_pose.get(), m_targetPose.get());
        m_outputTrajectory.accept(m_trajectory);
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        // if the target's moved more than 0.2 m since the last regen, generate the trajectory again
        if (m_targetPose.get().getTranslation().getDistance(m_lastRegenTarget.getTranslation()) > 0.2) {
            regenTrajectory();
        }
        
        PathPlannerState desiredState;
        ChassisSpeeds targetChassisSpeeds;
        // make sure the trajectory is not empty
        // make sure it's still time to be following the trajectory
        if (m_trajectory.getStates().size() != 0 && m_timer.get() < m_trajectory.getTotalTimeSeconds()) {
            double curTime = m_timer.get();
            desiredState = (PathPlannerState) m_trajectory.sample(curTime);
            double targetXSpeed = m_xController.calculate(m_pose.get().getX(), 
                desiredState.velocityMetersPerSecond * desiredState.holonomicRotation.getCos());
            double targetYSpeed = m_yController.calculate(m_pose.get().getY(), 
                desiredState.velocityMetersPerSecond * desiredState.holonomicRotation.getSin());
            double targetThetaSpeed = m_thetaController.calculate(m_pose.get().getRotation().getRadians(), 
                desiredState.velocityMetersPerSecond * desiredState.holonomicRotation.getRadians());
            targetChassisSpeeds = 
                ChassisSpeeds.fromFieldRelativeSpeeds(targetXSpeed, targetYSpeed, targetThetaSpeed, m_pose.get().getRotation());

            var alpha = ( 
                ((PathPlannerState) m_trajectory.sample(curTime + 0.01)).holonomicAngularVelocityRadPerSec
                - desiredState.holonomicAngularVelocityRadPerSec) / 0.01;

            targetChassisSpeeds.omegaRadiansPerSecond = alpha;
        } else {
            desiredState = new PathPlannerState();
            desiredState.holonomicRotation = m_targetPose.get().getRotation();
            desiredState.poseMeters = m_targetPose.get();
            desiredState.holonomicAngularVelocityRadPerSec = 0;
            double targetXSpeed = m_xController.calculate(m_pose.get().getX(), 
                desiredState.velocityMetersPerSecond * desiredState.holonomicRotation.getCos());
            double targetYSpeed = m_yController.calculate(m_pose.get().getY(), 
                desiredState.velocityMetersPerSecond * desiredState.holonomicRotation.getSin());
            double targetThetaSpeed = m_thetaController.calculate(m_pose.get().getRotation().getRadians(), 
                desiredState.velocityMetersPerSecond * desiredState.holonomicRotation.getRadians());
            targetChassisSpeeds = 
                ChassisSpeeds.fromFieldRelativeSpeeds(targetXSpeed, targetYSpeed, targetThetaSpeed, m_pose.get().getRotation());
        }           
    
        m_outputChassisSpeedsRobotRelative.accept(targetChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
