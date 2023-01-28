package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import static frc.robot.auton.Paths.PPPaths.oneMeter;
import static frc.robot.auton.Paths.PPPaths.rotateMove;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;

import static frc.robot.auton.Paths.PPPaths.diagonal;
import static frc.robot.auton.Paths.PPPaths.rotate;
import static frc.robot.auton.Paths.oneMeterForward;
import static frc.robot.auton.Paths.rotate90;
import static frc.robot.auton.Paths.moveDiagonal;


public final class AutonFactory {
    public static HashMap<String, Command> autonEventMap = new HashMap<>();
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");
    
    /*PATHING COMMANDS */
    public static CommandBase Move1MeterXAuto(Swerve swerve) {
        return swerve.getPPPathCmd(oneMeter,true);
    }

    public static CommandBase MoveDiagonal(Swerve swerve){
        return Commands.print("***********MOVING DIAGONAL***********")
        .andThen(swerve.getPPPathCmd(diagonal, true));
    }

    public static CommandBase Rotate(Swerve swerve){
        return Commands.print("***********ROTATING  90 DEGREES***********")
        .andThen(swerve.getPPPathCmd(rotate, true));
    }
    public static CommandBase RotateMove(Swerve swerve){
        return Commands.print("***********ROTATING 90 DEGREES AND MOVING***********")
        .andThen(swerve.getPPPathCmd(rotateMove, true));
    }

    public static CommandBase Move1Meter(Swerve swerve){
        return Commands.print("***********MOVING 1 METER***********")
        .andThen(swerve.getWPIPathCmd(oneMeterForward));
    }

    public static CommandBase Rotate90(Swerve swerve){
        return Commands.print("***********ROTATING 90 DEGREES***********")
        .andThen(swerve.getWPIPathCmd(rotate90));
    }

    public static CommandBase DiagonalMoving(Swerve swerve){
        return Commands.print("***********MOVING DIAGONAL***********")
        .andThen(swerve.getWPIPathCmd(moveDiagonal));
    } 
    
    /* EVENT COMMANDS */
    public static CommandBase TestEvent(Swerve swerve){
        return Commands.print("*****************TEST EVENT**************")
        .andThen(swerve.rotateAboutPoint(360));
    }

    /*FULL AUTON COMMAND*/
    public static CommandBase fullAuto(Swerve swerve, PathPlannerTrajectory traj){
        return(swerve.getFullAuto(traj));
    }
}
