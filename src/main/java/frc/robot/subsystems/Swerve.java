package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.SmartDashboardKeys;
import frc.robot.vision.AprilTagHelper;
import frc.lib.DashboardManager;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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
                // fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                //                     translation.getX(), 
                //                     translation.getY(), 
                //                     rotation, 
                //                     getYaw()
                //                 )
                //                 : 
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

    public CommandBase getSwerveControllerCommand(Trajectory trajectory) {
        var resetCommand = new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()));
        var autoSwerveCommand = new SwerveControllerCommand(
                trajectory,
                this::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                this::setModuleStates,
                this
        );
        return resetCommand.andThen(autoSwerveCommand);
    }

    @Override
    public void periodic(){
        for (var module : mSwerveMods) {
            module.periodic();
        }
        swerveOdometry.update(getYaw(), getModulePositions());  
        updateSmartDashboard();
    }

    public void updateSmartDashboard(){
    }
}