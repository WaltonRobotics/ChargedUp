package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.auton.Paths.PPPaths.oneMeter;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;

public final class AutonFactory {
    public static HashMap<String, Command> autonEventMap = new HashMap<>();
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");

    /* PATHING COMMANDS */
    public static CommandBase MoveOneMeter(SwerveSubsystem swerve) {
        return swerve.getFullAuto(oneMeter);
    }

    /* EVENT COMMANDS */
    public static CommandBase TestEvent(SwerveSubsystem swerve) {
        return Commands.print("*****************TEST EVENT**************")
                .andThen(swerve.rotateAboutPoint(360));
    }

    /* FULL AUTON COMMAND */
    public static CommandBase fullAuto(SwerveSubsystem swerve, PathPlannerTrajectory traj) {
        return (swerve.getFullAuto(traj));
    }
}
