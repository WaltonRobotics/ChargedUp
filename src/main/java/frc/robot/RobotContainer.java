package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auton.*;
import frc.robot.subsystems.*;
import frc.robot.auton.AutonChooser.AutonOption;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    // private final CommandXboxController driver = new CommandXboxController(0);
    private final Joystick driver = new Joystick(0);

    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem(AutonFactory.autonEventMap);

    /* Auton */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up autons
        mapAutonCommands();
        s_Swerve.setDefaultCommand(
            s_Swerve.teleopDriveCmd(
                () -> -driver.getRawAxis(1),
                // ()-> -1,
                // () -> -driver.getLeftY(),
                () -> -driver.getRawAxis(0),
                // () -> -driver.getLeftX(),
                () -> driver.getRawAxis(2),
                // driver.leftBumper()::getAsBoolean,
                () -> false,
                () -> true // openLoop
            )
        );

        initShuffleBoard();
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // driver.a().whileTrue(new RunCommand(() -> s_Swerve.followAprilTag(2, 0, true)));
        // driver.rightBumper().onTrue(new InstantCommand(() -> s_Swerve.handleAutoBalance()));
        // driver.x().onTrue(s_Swerve.rotateAboutPoint(90));
    }

    public void initShuffleBoard() {

    }

    public void mapAutonCommands() {
        AutonChooser.SetDefaultAuton(AutonOption.DO_NOTHING);
        AutonChooser.AssignAutonCommand(AutonOption.DO_NOTHING, AutonFactory.DoNothingAuto);
        AutonChooser.AssignAutonCommand(AutonOption.MOVE_FORWARD, AutonFactory.MoveOneMeter(s_Swerve));
        AutonChooser.AssignAutonCommand(AutonOption.THREE_PIECE2, AutonFactory.fullAuto(s_Swerve, Paths.PPPaths.threePiece2));
    }

    public void mapAutonEvents() {
        AutonFactory.autonEventMap.put("testEvent", AutonFactory.TestEvent(s_Swerve));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutonChooser.GetChosenAuton();
    }
}