package frc.robot.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.SwerveK;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.AutoBalance;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperState;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;

public final class AutonFactory {
    public static HashMap<String, Command> autonEventMap = new HashMap<>();
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");

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
        return new AutoBalance(swerve);
    }

    public static CommandBase oneConePark(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(2.5).withName("SS-Auto-TopCone");
        var clawCmd = claw.release().withName("ClawRelease");
        var ssResetCmd = superstructure.toState(SuperState.SAFE).withTimeout(2).withName("SS-Auto-Safe");
        var pathCmd = swerve.getPPSwerveAutonCmd(Paths.PPPaths.oneConePark).withTimeout(4.5).withName("Path1Cmd");
        // var pathCmd = swerve.getFullAuto(Paths.PPPaths.oneConePark).withInterruptBehavior(InterruptionBehavior.kCancelSelf).withTimeout(4.5).withName("Path1Cmd");

        return Commands.sequence(
            placeCmd,
            clawCmd,
            Commands.waitSeconds(1),
            ssResetCmd,
            Commands.print("===========BeforePathCmd==========="),
            // pathCmd,
            Commands.print("===========AfterPathCmd==========="),
            new AutoBalance(swerve).withName("autobalance"),
            Commands.print("===========AfterBalanceCmd===========")
        );
    }   

    public static CommandBase releaseClaw(TheClaw claw) {
        return claw.release();
    }

    public static CommandBase oneConeOneCube(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(3);
        var clawCmd = claw.release();
        var pathCmd = (superstructure.toState(SuperState.SAFE)).withTimeout(2)
                .andThen(swerve.getFullAuto(Paths.PPPaths.oneConeOneCube1))
                .andThen(superstructure.toState(SuperState.GROUND_PICK_UP).withTimeout(3)
                .andThen(swerve.getFullAuto(Paths.PPPaths.oneConeOneCube2))
                .andThen(superstructure.toState(SuperState.TOPCUBE).withTimeout(3)));

        return placeCmd.andThen(clawCmd).andThen(new WaitCommand(1.5).andThen(pathCmd).andThen(clawCmd));
    }

    public static CommandBase oneConeBalance(SwerveSubsystem swerve, Superstructure superstructure, TheClaw claw) {
        var placeCmd = superstructure.toState(SuperState.TOPCONE).withTimeout(3)
                .andThen(claw.release())
                .andThen(superstructure.toState(SuperState.SAFE));
        var driveCmd = new InstantCommand(() -> swerve.drive(-1, 0, 0, false, false)).withTimeout(4)
                .andThen(new InstantCommand(() -> swerve.drive(0, -1, 0, false, false)).withTimeout(1.5));
        return placeCmd.andThen(driveCmd).andThen(autoBalance(swerve));
    }
}
