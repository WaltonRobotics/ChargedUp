package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.WristK;
import frc.robot.auton.*;
import frc.lib.util.DashboardManager;
import frc.robot.subsystems.*;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.AprilTagChooser;
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
    private final CommandXboxController manipulator = new CommandXboxController(1);

    public final AprilTagCamera vision = new AprilTagCamera();
    public final SwerveSubsystem swerve = new SwerveSubsystem(autonEventMap, vision);
    public final TiltSubsystem tilt = new TiltSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final TheClaw claw = new TheClaw();
    public final LEDSubsystem leds = new LEDSubsystem();

    /* Subsystems */
    public final Superstructure superstructure = new Superstructure(tilt, elevator, wrist, claw);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up autons
        mapAutonCommands();
        mapTrajectories();
        mapAprilTagPoints();
        // addPathChoices();
        // addAprilTagChoices();
        swerve.setDefaultCommand(
                swerve.teleopDriveCmd(
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> driver.getRightX(),
                        driver.leftBumper()::getAsBoolean,
                        () -> true // openLoop
                ));
        elevator.setDefaultCommand(elevator.teleOpCmd(() -> -manipulator.getLeftY()));
        tilt.setDefaultCommand(tilt.teleopCmd(() -> manipulator.getRightY()));
        wrist.setDefaultCommand(wrist.teleopCmd(() -> manipulator.getLeftX()));

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
        driver.leftBumper().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        driver.leftTrigger()
                .whileTrue(new InstantCommand(() -> swerve.drive(-0.5, 0, 0, true, true)));
        driver.x().onTrue(swerve.rotateAboutPoint(90));
        driver.b().onTrue(new InstantCommand(() -> swerve.resetOdometryPose()));
        driver.leftBumper().whileTrue(swerve.autoScore());
        
        manipulator.povLeft().onTrue(new InstantCommand(() -> leds.handle(0))); // cone
        manipulator.povRight().onTrue(new InstantCommand(() -> leds.handle(1))); // cube
        manipulator.rightTrigger()
                .whileTrue(claw.autoGrab(true));
        manipulator.rightTrigger().onFalse(claw.release());
        manipulator.x().whileTrue(wrist.toAngle(0));
        manipulator.b().whileTrue(elevator.toHeight(0.3));
        manipulator.a().whileTrue(tilt.toAngle(15));
        manipulator.y().whileTrue(tilt.toAngle(0));
        manipulator.rightBumper().whileTrue(wrist.toAngle(WristK.kMaxAngleDegrees));
    }
    public void mapAutonCommands() {
        AutonChooser.SetDefaultAuton(AutonOption.DO_NOTHING);
        AutonChooser.AssignAutonCommand(AutonOption.DO_NOTHING, AutonFactory.DoNothingAuto);
        AutonChooser.AssignAutonCommand(AutonOption.MOVE_FORWARD,
                AutonFactory.WaltonPPAuto(swerve, oneMeter));
        AutonChooser.AssignAutonCommand(AutonOption.THREE_PIECE2,
                AutonFactory.WaltonPPAuto(swerve, threePiece2));
        AutonChooser.AssignAutonCommand(AutonOption.TWO_PIECE_PAUSE,
                swerve.getFullAuto(twoPiece).withName("TwoPiecePauseAuto"));
        AutonChooser.AssignAutonCommand(AutonOption.THREE_PIECE3,
                AutonFactory.WaltonPPAuto(swerve, threePiece3));
        AutonChooser.AssignAutonCommand(AutonOption.TWO_PIECE_BALANCE,
                AutonFactory.WaltonPPAuto(swerve, twoPieceBalance));
        AutonChooser.AssignAutonCommand(AutonOption.ONE_PIECE,
                AutonFactory.WaltonPPAuto(swerve, onePiece));
    }

    public void mapTrajectories() {
        PathChooser.AssignTrajectory(PathOption.RED_NON_BUMPY, PPAutoscoreClass.redNotBumpy);
        PathChooser.SetDefaultPath(PathOption.RED_NON_BUMPY);
        PathChooser.AssignTrajectory(PathOption.RED_BUMPY, PPAutoscoreClass.redBumpy);
        PathChooser.AssignTrajectory(PathOption.BLUE_BUMPY, PPAutoscoreClass.blueBumpy);
        PathChooser.AssignTrajectory(PathOption.BLUE_NON_BUMPY, PPAutoscoreClass.blueNotBumpy);
    }

    public void mapAprilTagPoints() {
        AprilTagChooser.AssignPoint(AprilTagOption.TAG_1, ReferencePoints.tag1);
        AprilTagChooser.SetDefaultAprilTag(AprilTagOption.TAG_1);
        AprilTagChooser.AssignPoint(AprilTagOption.TAG_2, ReferencePoints.tag2);
        AprilTagChooser.AssignPoint(AprilTagOption.TAG_3, ReferencePoints.tag3);
        AprilTagChooser.AssignPoint(AprilTagOption.TAG_6, ReferencePoints.tag6);
        AprilTagChooser.AssignPoint(AprilTagOption.TAG_7, ReferencePoints.tag7);
        AprilTagChooser.AssignPoint(AprilTagOption.TAG_8, ReferencePoints.tag8);
    }

    public void mapAutonEvents() {
        autonEventMap.put("testEvent", AutonFactory.TestEvent(swerve));
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    public void turnOffRumble() {
        driver.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutonChooser.GetChosenAuton();
    }

    // public void addPathChoices() {
    // DashboardManager.addTab("Path Chooser");
    // if(DriverStation.getAlliance().equals(Alliance.Red)) {
    // DashboardManager.addTabItem("Path Chooser", "bumper side",
    // PathOption.RED_BUMPY);
    // DashboardManager.addTabItem("Path Chooser", "no bumper side",
    // PathOption.RED_NON_BUMPY);
    // }
    // else if(DriverStation.getAlliance().equals(Alliance.Blue)) {
    // DashboardManager.addTabItem("Path Chooser", "bumper side",
    // PathOption.BLUE_BUMPY);
    // DashboardManager.addTabItem("Path Chooser", "no bumper side",
    // PathOption.BLUE_NON_BUMPY);
    // }
    // }

    // public void addAprilTagChoices() {
    // DashboardManager.addTab("AprilTag Chooser");
    // if(DriverStation.getAlliance().equals(Alliance.Red)) {
    // DashboardManager.addTabItem("AprilTag Chooser", "tag 1",
    // AprilTagOption.TAG_1);
    // DashboardManager.addTabItem("AprilTag Chooser", "tag 2",
    // AprilTagOption.TAG_2);
    // DashboardManager.addTabItem("AprilTag Chooser", "tag 3",
    // AprilTagOption.TAG_3);
    // }
    // else if(DriverStation.getAlliance().equals(Alliance.Blue)) {
    // DashboardManager.addTabItem("AprilTag Chooser", "tag 6",
    // AprilTagOption.TAG_6);
    // DashboardManager.addTabItem("AprilTag Chooser", "tag 7",
    // AprilTagOption.TAG_7);
    // DashboardManager.addTabItem("AprilTag Chooser", "tag 8",
    // AprilTagOption.TAG_8);
    // }
    // }
}