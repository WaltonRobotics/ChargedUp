package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.auton.Paths.TestPaths;

public final class AutonFactory {
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");
    
    public static CommandBase Move1MeterXAuto(Swerve swerve) {
        return swerve.getAutonSwerveControllerCommand(TestPaths.oneMeter,true);
    }

    public static CommandBase MoveDiagonal(Swerve swerve){
        return Commands.print("***********MOVING DIAGONAL***********")
        .andThen(swerve.getAutonSwerveControllerCommand(TestPaths.diagonal, true));
    }

    public static CommandBase Rotate(Swerve swerve){
        return Commands.print("***********ROTATING  90 DEGREES***********")
        .andThen(swerve.getAutonSwerveControllerCommand(TestPaths.rotate, true));
    }
    public static CommandBase RotateMove(Swerve swerve){
        return Commands.print("***********ROTATING 90 DEGREES AND MOVING***********")
        .andThen(swerve.getAutonSwerveControllerCommand(TestPaths.rotateMove, true));
    }

    public static CommandBase Move1Meter(Swerve swerve){
        return Commands.print("***********MOVING 1 METER***********")
        .andThen(swerve.getSwerveControllerCommand(Paths.oneMeterForward));
    }

    public static CommandBase Rotate90(Swerve swerve){
        return Commands.print("***********ROTATING 90 DEGREES***********")
        .andThen(swerve.getSwerveControllerCommand(Paths.rotate90));
    }

    public static CommandBase DiagonalMoving(Swerve swerve){
        return Commands.print("***********MOVING DIAGONAL***********")
        .andThen(swerve.getSwerveControllerCommand(Paths.moveDiagonal));
    } 
}
