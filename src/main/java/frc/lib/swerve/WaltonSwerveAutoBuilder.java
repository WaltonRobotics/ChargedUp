package frc.lib.swerve;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class WaltonSwerveAutoBuilder extends SwerveAutoBuilder {

    private final SwerveDriveKinematics kinematics;
    private final PIDConstants translationConstants;
    private final PIDConstants rotationConstants;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Subsystem[] driveRequirements;

    public WaltonSwerveAutoBuilder(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        SwerveDriveKinematics kinematics,
        PIDConstants translationConstants,
        PIDConstants rotationConstants,
        Consumer<SwerveModuleState[]> outputModuleStates,
        Map<String, Command> eventMap,
        boolean useAllianceColor,
        Subsystem... driveRequirements
    ) {
        super(poseSupplier, resetPose, kinematics, translationConstants, rotationConstants, outputModuleStates, eventMap,
                useAllianceColor, driveRequirements);

        this.kinematics = kinematics;
        this.translationConstants = translationConstants;
        this.rotationConstants = rotationConstants;
        this.outputModuleStates = outputModuleStates;
        this.driveRequirements = driveRequirements;

    }

    public CommandBase followPath(Supplier<PathPlannerTrajectory> trajSupplier) {
        return new WaltonPPSwerveControllerCommand(
            trajSupplier,
            poseSupplier,
            kinematics,
            pidControllerFromConstants(translationConstants),
            pidControllerFromConstants(translationConstants),
            pidControllerFromConstants(rotationConstants),
            outputModuleStates,
            useAllianceColor,
            driveRequirements);
    }
    
}
