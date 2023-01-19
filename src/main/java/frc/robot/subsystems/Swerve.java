package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.vision.AprilTagHelper;
import frc.lib.util.DashboardManager;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private Pigeon2 gyro;
    private ProfiledPIDController thetaController;


    public Swerve() {
        DashboardManager.addTab(this);
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "Canivore");
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule("Front Left", 0, Constants.Swerve.Mod0.constants),
            new SwerveModule("Front Right", 1, Constants.Swerve.Mod1.constants),
            new SwerveModule("Rear Left", 2, Constants.Swerve.Mod2.constants),
            new SwerveModule("Rear Right", 3, Constants.Swerve.Mod3.constants)
        };

        Timer.delay(.250);
        for (var mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
        

        thetaController =
              new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 
                    0,
                    0,
                    Constants.AutoConstants.kThetaControllerConstraints
                    );

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        swerveOdometry = new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics,
                getYaw(),
                getModulePositions()
                );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : 
                                new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    } 

    public void drive(double x, double y, double rotation, boolean fieldRelative, boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates = 
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                x, 
                                y, 
                                rotation, 
                                getYaw()
                            )
                            : 
                            new ChassisSpeeds(
                                x, 
                                y, 
                                rotation)
                            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    //side to side  
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Use Apriltag vision to balance robot
     * on the charging station
     */
    public void handleAutoBalance(){
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

    public void followAprilTag(double goalDistance, boolean isFieldRelative){
        if(AprilTagHelper.hasTargets()){
            PhotonTrackedTarget target = AprilTagHelper.getBestTarget();
            double headingError = target.getYaw();

            double distance =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                VisionConstants.kCameraHeight,
                                VisionConstants.kTargetHeight,
                                0,
                                Units.degreesToRadians(target.getPitch())
            );
            Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(headingError));

            double xRate = thetaController.calculate(translation.getX(), 0);
            double yRate = thetaController.calculate(translation.getY(),goalDistance);
            double turnRate = thetaController.calculate(headingError,0);
            
            drive(xRate, yRate, turnRate, isFieldRelative, true);
        }
    }

    public ProfiledPIDController getThetaController(){
        return thetaController;
    }

    // public CommandBase getSwerveControllerCommand(Trajectory trajectory) {
    //     var resetCommand = new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()));
    //     var autoSwerveCommand = new SwerveControllerCommand(
    //             trajectory,
    //             this::getPose,
    //             Constants.Swerve.swerveKinematics,
    //             new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //             new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //             thetaController,
    //             this::setModuleStates,
    //             this
    //     );
    //     return resetCommand.andThen(autoSwerveCommand);
    // }

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getInitialHolonomicPose());
            }
            }),
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            )
        );
    }

    @Override
    public void periodic(){
        for (var module : mSwerveMods) {
            module.periodic();
        }
        swerveOdometry.update(getYaw(), getModulePositions());  
    }
}