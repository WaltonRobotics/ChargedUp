package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auton.*;
import frc.lib.util.DashboardManager;
import frc.robot.subsystems.*;
import frc.robot.vision.AprilTagChooser;
import frc.robot.vision.AprilTagHelper;
import frc.robot.vision.PathChooser;
import frc.robot.vision.AprilTagChooser.AprilTagOption;
import frc.robot.vision.PathChooser.PathOption;
import frc.robot.auton.AutonChooser.AutonOption;
import frc.robot.auton.Paths.PPAutoscoreClass;
import frc.robot.auton.Paths.ReferencePoints;

import static frc.robot.auton.AutonFactory.autonEventMap;
import static frc.robot.auton.Paths.PPPaths.*;

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
    private final CommandXboxController driver = new CommandXboxController(0);

    private final AprilTagHelper m_apriltagHelper = new AprilTagHelper();

    /* Subsystems */
    public final SwerveSubsystem s_Swerve = new SwerveSubsystem(autonEventMap, m_apriltagHelper);
    private final Elevator s_Elevator = new Elevator();
    /* Drive Controls */



    

    /* Auton */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up autons
        mapAutonCommands();
        // mapTrajectories();
        // mapAprilTagPoints();
        s_Swerve.setDefaultCommand(
            s_Swerve.teleopDriveCmd(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                driver.leftBumper()::getAsBoolean,
                () -> true // openLoop
            )
        );
        initShuffleBoard();
        DashboardManager.addTab("TeleSwerve");
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
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driver.a().whileTrue(new RunCommand(() -> s_Swerve.followAprilTag(2, 0, true)));
        driver.rightBumper().onTrue(new InstantCommand(() -> s_Swerve.handleAutoBalance()));
        driver.leftTrigger().whileTrue(new InstantCommand(() -> s_Swerve.drive(-0.5, 0, 0, true, true)));
        driver.x().onTrue(s_Swerve.rotateAboutPoint(90));
        driver.b().onTrue(new InstantCommand(() -> s_Swerve.resetOdometryPose()));
        driver.leftBumper().whileTrue(s_Swerve.autoScore());
    }

    public void initShuffleBoard() {
        SmartDashboard.putNumber("PX Controller", Constants.AutoConstants.kPXController);
        SmartDashboard.putNumber("PY Controller", Constants.AutoConstants.kPYController);
        SmartDashboard.putNumber("PTheta Controller", Constants.AutoConstants.kPThetaController);
    }

    public void mapAutonCommands() {
        AutonChooser.SetDefaultAuton(AutonOption.DO_NOTHING);
        AutonChooser.AssignAutonCommand(AutonOption.DO_NOTHING, AutonFactory.DoNothingAuto);
        AutonChooser.AssignAutonCommand(AutonOption.MOVE_FORWARD, AutonFactory.MoveOneMeter(s_Swerve));
        AutonChooser.AssignAutonCommand(AutonOption.THREE_PIECE2, AutonFactory.fullAuto(s_Swerve, threePiece2));
        AutonChooser.AssignAutonCommand(AutonOption.TWO_PIECE_PAUSE, s_Swerve.getFullAuto(twoPiece).withName("TwoPiecePauseAuto"));
        AutonChooser.AssignAutonCommand(AutonOption.THREE_PIECE3, AutonFactory.fullAuto(s_Swerve, threePiece3));
        AutonChooser.AssignAutonCommand(AutonOption.TWO_PIECE_BALANCE, AutonFactory.fullAuto(s_Swerve, twoPieceBalance));
        AutonChooser.AssignAutonCommand(AutonOption.ONE_PIECE, AutonFactory.fullAuto(s_Swerve, onePiece));        
    }

    public void mapTrajectories() {
        
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            PathChooser.AssignTrajectory(PathOption.RED_BUMPY, PPAutoscoreClass.redBumpy);
            PathChooser.AssignTrajectory(PathOption.RED_NON_BUMPY, PPAutoscoreClass.redNotBumpy);
        } else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            PathChooser.AssignTrajectory(PathOption.BLUE_BUMPY, PPAutoscoreClass.blueBumpy);
            PathChooser.AssignTrajectory(PathOption.BLUE_NON_BUMPY, PPAutoscoreClass.blueNotBumpy);  
        }       
    }

    public void mapAprilTagPoints() {
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            AprilTagChooser.AssignPoint(AprilTagOption.TAG_1, ReferencePoints.tag1);
            AprilTagChooser.AssignPoint(AprilTagOption.TAG_2, ReferencePoints.tag2);
            AprilTagChooser.AssignPoint(AprilTagOption.TAG_3, ReferencePoints.tag3);
        } else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            AprilTagChooser.AssignPoint(AprilTagOption.TAG_6, ReferencePoints.tag6);
            AprilTagChooser.AssignPoint(AprilTagOption.TAG_7, ReferencePoints.tag7);
            AprilTagChooser.AssignPoint(AprilTagOption.TAG_8, ReferencePoints.tag8);
        }
    }

    public void mapAutonEvents() {
        autonEventMap.put("testEvent", AutonFactory.TestEvent(s_Swerve));
        driver.a()
            .onTrue(s_Elevator.setLiftTarget(0.3))
            .onFalse(s_Elevator.setLiftTarget(0));
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
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