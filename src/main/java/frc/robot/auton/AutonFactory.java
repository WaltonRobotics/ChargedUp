package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import static frc.robot.auton.Paths.OneMeter.oneMeter;

public final class AutonFactory {
    public static final CommandBase DoNothingAuto = Commands.print("Doing Nothing!!!!!!!!!!!");

    public static CommandBase Move1MeterXAuto(Swerve swerve) {
        return swerve.getSwerveControllerCommand(oneMeter);
    }

    // public static CommandBase MoveDiagonal(Swerve swerve){
    //     return swerve.getSwerveControllerCommand(moveDiagonal);
    // }
    

}
