package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.DashboardManager;
import frc.robot.auton.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //public static final SendableChooser<Autons> autonChooser = new SendableChooser<>();
    
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        DashboardManager.addTab("TeleSwerve");
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                driver::getLeftY, 
                driver::getLeftX, 
                driver::getRightX, 
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
    // Auton chooser
        // Arrays.stream(Autons.values()).forEach(n -> autonChooser.addOption(n.name(), n));
        // autonChooser.setDefaultOption("DO_NOTHING", Autons.DO_NOTHING);
        // SmartDashboard.putData("Auton Selector", autonChooser);
        // SmartDashboard.putNumber("Goal Distance", 0.0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    //     Autons routine = autonChooser.getSelected();
    //     return routine.getCommandGroup();
    // }
}