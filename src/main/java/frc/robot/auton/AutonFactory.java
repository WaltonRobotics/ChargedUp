package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.auton.Paths.TestPaths;


public final class AutonFactory {
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");

    public static CommandBase Move1MeterXAuto(Swerve swerve) {
        System.out.println("*********MOVING 1 METER**********");
        return swerve.getAutonSwerveControllerCommand(TestPaths.oneMeter);
    }

    public static CommandBase MoveDiagonal(Swerve swerve){
        System.out.println("***********MOVING DIAGONAL***********");
        return swerve.getSwerveControllerCommand(TestPaths.diagonal);
    }
    public static CommandBase Rotate(Swerve swerve){
        System.out.println("***********ROTATING 90 DEGREES***********");
        return swerve.getSwerveControllerCommand(TestPaths.rotate);
    }
    public static CommandBase RotateMove(Swerve swerve){
        System.out.println("***********ROTATING 90 DEGREES AND MOVING***********");
        return swerve.getSwerveControllerCommand(TestPaths.rotateMove);
    }

}
