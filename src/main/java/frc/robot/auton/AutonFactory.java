package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.auton.Paths.Diagonal;
import frc.robot.auton.Paths.OneMeter;

public final class AutonFactory {
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");

    public static CommandBase Move1MeterXAuto(Swerve swerve) {
        System.out.println("*********ROTATING**********");
        return swerve.getAutonSwerveControllerCommand(OneMeter.oneMeter);
    }

    public static CommandBase MoveDiagonal(Swerve swerve){
        System.out.println("***********MOVING***********");
        return swerve.getSwerveControllerCommand(Diagonal.diagonal);
    }

}
