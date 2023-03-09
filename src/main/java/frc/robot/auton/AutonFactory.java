package frc.robot.auton;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.AutoBalance;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperState;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;

public final class AutonFactory {
    public static HashMap<String, Command> autonEventMap = new HashMap<>();
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");
    public static Translation2d position = new Translation2d();

    /* EVENT COMMANDS */
    public static CommandBase TestEvent(SwerveSubsystem swerve) {
        return Commands.print("*****************TEST EVENT**************")
                .andThen(swerve.rotateAboutPoint(360));
    }

    /* FULL AUTON COMMAND */
    public static CommandBase fullAuto(SwerveSubsystem swerve, PathPlannerTrajectory traj) {
        return (swerve.getFullAuto(traj));
    }

    public static CommandBase autoBalance(SwerveSubsystem swerve){
        return new AutoBalance(swerve, false);
    }

    public static CommandBase oneConePark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2.5).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        // var pathCmd = swerve.getPPSwerveAutonCmd(Paths.PPPaths.oneConePark).withName("Path1Cmd");
        var getPosition = new InstantCommand(()-> position = swerve.getPose().getTranslation());
        // var pathCmd = swerve.getFullAuto(Paths.PPPaths.oneConePark).withInterruptBehavior(InterruptionBehavior.kCancelSelf).withTimeout(4.5).withName("Path1Cmd");
        var driveCmd = swerve.driveOneDirection(false).until(
            ()-> swerve.atDistance(position, 2));

        return Commands.sequence(
            placeCmd,
            clawCmd,
            Commands.waitSeconds(1),
            ssResetCmd,
            Commands.print("===========BeforePathCmd==========="),
            getPosition,
            driveCmd,
            Commands.print("===========AfterPathCmd==========="),
            new AutoBalance(swerve, false).withName("autobalance"),
            Commands.print("===========AfterBalanceCmd==========="),
            new AutoBalance(swerve, false)
        );
    }

    public static CommandBase oneConeBack(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(3).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");

        return Commands.sequence(
            placeCmd,
            Commands.waitSeconds(0.2),
            clawCmd,
            Commands.waitSeconds(0.4),
            ssResetCmd,
            new AutoBalance(swerve, true)
        );
    }
    
    public static CommandBase manualStateTest(TiltSubsystem tilt, ElevatorSubsystem elev, WristSubsystem wrist) {
        return Commands.none();
    }

    public static CommandBase oneConeAround(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2.5).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var backCmd = swerve.driveOneDirection(true).withTimeout(3);
        var strafeCmd = swerve.driveSide(true).withTimeout(2);

        return Commands.sequence(
            placeCmd,
            clawCmd,
            ssResetCmd,
            backCmd,
            strafeCmd,
            new AutoBalance(swerve, false)
        );
    }

    public static CommandBase oneCubeAround(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCUBE).withTimeout(2.5).withName("SS-Auto-TopCube");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var backCmd = swerve.driveOneDirection(true).withTimeout(3);
        var strafeCmd = swerve.driveSide(false).withTimeout(2);

        return Commands.sequence(
            placeCmd,
            clawCmd,
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
