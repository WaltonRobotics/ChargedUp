package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.DashboardManager;
import frc.robot.auton.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.auton.AutonChooser.AutonOption;
import frc.robot.auton.Paths.PPPaths;

import static frc.robot.auton.AutonFactory.autonEventMap;

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
    private final Swerve s_Swerve = new Swerve(autonEventMap);

    /*Auton */

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
        driver.a().whileTrue(new RunCommand(() -> s_Swerve.followAprilTag(2, 0, true)));
        driver.rightBumper().onTrue(new InstantCommand(() -> s_Swerve.handleAutoBalance()));
        driver.x().onTrue(s_Swerve.rotateAboutPoint(90));
    }

    public void initShuffleBoard(){
        
    }

    public void mapAutonCommands(){
        AutonChooser.SetDefaultAuton(AutonOption.DO_NOTHING);
        AutonChooser.SetAutonCommand(AutonOption.DO_NOTHING, AutonFactory.DoNothingAuto);
        AutonChooser.SetAutonCommand(AutonOption.MOVE_FORWARD, AutonFactory.Move1MeterXAuto(s_Swerve));
        // AutonChooser.SetAutonCommand(AutonOption.MOVE_DIAGONAL, AutonFactory.MoveDiagonal(s_Swerve));
        AutonChooser.SetAutonCommand(AutonOption.MOVE_DIAGONAL, AutonFactory.fullAuto(s_Swerve, PPPaths.diagonal));
        AutonChooser.SetAutonCommand(AutonOption.THREE_PIECE2, AutonFactory.fullAuto(s_Swerve, PPPaths.threePiece));
        AutonChooser.SetAutonCommand(AutonOption.ROTATE, AutonFactory.Rotate(s_Swerve));
        AutonChooser.SetAutonCommand(AutonOption.ROTATE_MOVE, AutonFactory.RotateMove(s_Swerve));
        AutonChooser.SetAutonCommand(AutonOption.MOVE_FORWARD_1_METER, AutonFactory.Move1Meter(s_Swerve));
        AutonChooser.SetAutonCommand(AutonOption.ROTATE_90, AutonFactory.Rotate90(s_Swerve));       
        AutonChooser.SetAutonCommand(AutonOption.MOVE_DIAGONALLY, AutonFactory.DiagonalMoving(s_Swerve));   
    }

    public void mapAutonEvents(){
        autonEventMap.put("testEvent", AutonFactory.TestEvent(s_Swerve));
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