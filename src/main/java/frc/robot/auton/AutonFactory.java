package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.AutoBalance;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperState;

import java.util.HashMap;

import com.pathplanner.lib.PathPoint;

public final class AutonFactory {
    public static HashMap<String, Command> autonEventMap = new HashMap<>();
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");
    public static Translation2d position = new Translation2d();
   
    public static CommandBase oneConePark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(3).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var driveCmd = swerve.driveOneDirection(true).withTimeout(3);
        var strafeCmd = swerve.driveSide(true).withTimeout(1.6);

        return Commands.sequence(
            superstructure.dumbReset(),
            placeCmd,
            clawCmd,
            Commands.waitSeconds(1),
            ssResetCmd,
            driveCmd,
            strafeCmd,
            new AutoBalance(swerve, false).withName("autobalance")
        );
    }

    public static CommandBase oneConeBack(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(3.35).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");

        return Commands.sequence(
            superstructure.dumbReset(),
            placeCmd,
            Commands.waitSeconds(0.2),
            clawCmd,
            Commands.waitSeconds(0.4),
            ssResetCmd,
            new AutoBalance(swerve, true)
        );
    }

    // public static CommandBase oneConeAround(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
    //     var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2.5).withName("SS-Auto-TopCone");
    //     var clawCmd = claw.release().withName("ClawRelease");
    //     var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
    //     var backCmd = swerve.driveOneDirection(true).withTimeout(3);
    //     var strafeCmd = swerve.driveSide(true).withTimeout(2);

    //     return Commands.sequence(
    //         placeCmd,
    //         clawCmd,
    //         ssResetCmd,
    //         backCmd,
    //         strafeCmd,
    //         new AutoBalance(swerve, false)
    //     );
    // }

    public static CommandBase oneCubeAround(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCUBE).withTimeout(2.5).withName("SS-Auto-TopCube");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var backCmd = swerve.driveOneDirection(true).withTimeout(3);
        var strafeCmd = swerve.driveSide(false).withTimeout(1.25);

        return Commands.sequence(
            superstructure.dumbReset(),
            placeCmd,
            Commands.waitSeconds(0.4),
            clawCmd,
            Commands.waitSeconds(0.4),
            ssResetCmd,
            backCmd,
            strafeCmd,
            new AutoBalance(swerve, false)
        );
    }

    public static CommandBase releaseClaw(TheClaw claw) {
        return claw.release();
    }
}
