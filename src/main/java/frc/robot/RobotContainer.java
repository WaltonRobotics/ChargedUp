package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.DashboardManager;
import frc.robot.auton.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.auton.AutonManager.AutonOption;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Set up autons
        mapAutonCommands();
        DashboardManager.addTab("TeleSwerve");
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                ()-> -driver.getLeftY(), 
                ()-> -driver.getLeftX(), 
                ()-> -driver.getRightX(), 
                driver.leftBumper()::getAsBoolean
            )
        );

        initShuffleBoard();
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driver.a().whileTrue(new RunCommand(() -> s_Swerve.followAprilTag(1, true)));
        driver.rightBumper().onTrue(new InstantCommand(() -> s_Swerve.handleAutoBalance()));
    }

    public void initShuffleBoard(){
    }

    public void mapAutonCommands(){
        AutonManager.SetDefaultAuton(AutonOption.DO_NOTHING);
        AutonManager.SetAutonCommand(AutonOption.DO_NOTHING, AutonFactory.DoNothingAuto);
        // AutonManager.SetAutonCommand(AutonOption.MOVE_FORWARD, AutonFactory.Move1MeterXAuto(s_Swerve));
        // AutonManager.SetAutonCommand(AutonOption.MOVE_DIAGONAL, AutonFactory.MoveDiagonal(s_Swerve));
        AutonManager.SetAutonCommand(AutonOption.ROTATE, AutonFactory.Rotate(s_Swerve));
        AutonManager.SetAutonCommand(AutonOption.ROTATE_MOVE, AutonFactory.RotateMove(s_Swerve));
        AutonManager.SetAutonCommand(AutonOption.MOVE_FORWARD_1_METER, AutonFactory.Move1Meter(s_Swerve));
        AutonManager.SetAutonCommand(AutonOption.ROTATE_90, AutonFactory.Rotate90(s_Swerve));       
        AutonManager.SetAutonCommand(AutonOption.MOVE_DIAGONALLY, AutonFactory.DiagonalMoving(s_Swerve)); 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutonManager.GetChosenAuton();
    }
}