package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.TiltK;
import frc.robot.Constants.WristK;
import frc.robot.auton.*;
import frc.lib.util.DashboardManager;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.ScoringStates;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.AprilTagChooser;
import frc.robot.vision.PathChooser;
import frc.robot.vision.AprilTagChooser.AprilTagOption;
import frc.robot.vision.PathChooser.PathOption;
import frc.robot.auton.AutonChooser.AutonOption;
import frc.robot.auton.Paths.PPAutoscoreClass;
import frc.robot.auton.Paths.ReferencePoints;
import frc.robot.auton.Paths.ReferencePoints.ScoringPlaces;
import frc.robot.auton.Paths.ReferencePoints.ScoringPoints;

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
                driver.x().whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCube2));
                driver.y().whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCoopCone6));
                driver.b().whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCube8));
                driver.leftBumper().whileTrue(swerve.autoScore());

                // manipulator.povLeft().onTrue(new InstantCommand(() -> leds.handle(0))); //
                // cone
                // manipulator.povRight().onTrue(new InstantCommand(() -> leds.handle(1))); //
                // cube
                manipulator.rightTrigger()
                                .whileTrue(claw.autoGrab(true));
                manipulator.leftTrigger().onTrue(claw.release());
                manipulator.rightBumper().onTrue(claw.grab());

                // manipulator.povUp().whileTrue(superstructure.toState(ScoringStates.TOPCUBE).andThen(claw.release()));
                // manipulator.povLeft().whileTrue(superstructure.toState(ScoringStates.MIDCUBE).andThen(claw.release()));
                // manipulator.y().whileTrue(superstructure.toState(ScoringStates.TOPCONE).andThen(claw.release()));
                // manipulator.x().whileTrue(superstructure.toState(ScoringStates.MIDCONE).andThen(claw.release()));
                // manipulator.povDown().whileTrue(superstructure.toState(ScoringStates.BOT).andThen(claw.release()));
                // manipulator.a().whileTrue(superstructure.toState(ScoringStates.BOT).andThen(claw.release()));
                // manipulator.povRight().whileTrue(superstructure.toState(ScoringStates.SUBSTATION_PICK_UP).andThen(claw.release()));
                
                manipulator.povUp().whileTrue((tilt.toAngle(29)) // top cube
                .alongWith(elevator.toHeight(0.577731))
                .alongWith(wrist.toAngle(7.010672))
                .andThen(claw.release()));

                manipulator.povLeft().whileTrue((tilt.toAngle(TiltK.kMidCubeAngleDegrees)) // mid cube
                                .alongWith(elevator.toHeight(ElevatorK.kMidCubeHeightM))
                                .alongWith(wrist.toAngle(WristK.kMidCubeAngleDegrees))
                                .andThen(claw.release()));

                manipulator.y().whileTrue((elevator.toHeight(0.723719)) // top cone
                                .alongWith(tilt.toAngle(27.7))
                                .alongWith(wrist.toAngle(-10))
                                .andThen(claw.release()));

                manipulator.x().whileTrue((tilt.toAngle(TiltK.kMidConeAngleDegrees)) // mid cone
                                .alongWith(elevator.toHeight(ElevatorK.kMidConeHeightM))
                                .alongWith(wrist.toAngle(WristK.kMidConeAngleDegrees))
                                .andThen(claw.release()));

                manipulator.povDown().whileTrue((tilt.toAngle(TiltK.kBotAngleDegrees)) // bottom
                                .alongWith(elevator.toHeight(ElevatorK.kBotHeightMeters))
                                .alongWith(wrist.toAngle(WristK.kBotAngleDegrees))
                                .andThen(claw.release()));
                manipulator.a().whileTrue((tilt.toAngle(TiltK.kBotAngleDegrees)) // bottom
                                .alongWith(elevator.toHeight(ElevatorK.kBotHeightMeters))
                                .alongWith(wrist.toAngle(WristK.kBotAngleDegrees))
                                .andThen(claw.release()));

                manipulator.povRight().whileTrue((tilt.toAngle(TiltK.kSubstationAngleDegrees)) // substation
                                .alongWith(elevator.toHeight(ElevatorK.kSubstationHeightM))
                                .alongWith(wrist.toAngle(0))
                                .alongWith(claw.autoGrab(false)));

                manipulator.leftBumper().whileTrue((wrist.toAngle(-18)) // to zero
                                .alongWith(elevator.toHeight(ElevatorK.kMinHeightMeters)
                                                .andThen(tilt.toAngle(0))));

                // manipulator.a().whileTrue(wrist.toFlat());
                // manipulator.x().whileTrue(wrist.toAngle(0));
                // manipulator.b().whileTrue(elevator.toHeight(0.3));
                // manipulator.a().whileTrue(tilt.toAngle(15));
                // manipulator.y().whileTrue(tilt.toAngle(0));
                manipulator.rightBumper().whileTrue(wrist.toAngle(WristK.kMaxAngleDegrees));
        }

        public void mapAutonCommands() {
               AutonChooser.AssignAutonCommand(AutonOption.BACK_OUT, AutonFactory.WaltonPPAuto(swerve, backOut));
               AutonChooser.AssignAutonCommand(AutonOption.CUBE_CONE_1, AutonFactory.WaltonPPAuto(swerve, cubeConeNonBumper));
               AutonChooser.AssignAutonCommand(AutonOption.CUBE_CONE_2, AutonFactory.WaltonPPAuto(swerve, cubeConeBumper));
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
                autonEventMap.put("score cube", superstructure.toState(ScoringStates.TOPCUBE).andThen(claw.release()));
                autonEventMap.put("score cone", superstructure.toState(ScoringStates.TOPCONE).andThen(claw.release()));
                autonEventMap.put("ground pickup", superstructure.toState(ScoringStates.GROUND_PICK_UP)
                                .andThen(claw.autoGrab(false)));
                autonEventMap.put("autobalance", swerve.autoBalance());
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