package frc.robot.autos;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.autos.Paths.UTest.uPath;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){
        addCommands(
           s_Swerve.getSwerveControllerCommand(uPath)
        );
    }
}