package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

        private final double waitTimeSecondsWrist = .4;
        private final double waitTimeSecondsEle = .1;

        /* Subsystems */
        public final Superstructure superstructure = new Superstructure(tilt, elevator, wrist, claw);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
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
                driver.rightTrigger().onTrue(new InstantCommand(() -> swerve.resetModsToAbs()));
                driver.leftTrigger()
                                .whileTrue(new InstantCommand(() -> swerve.drive(-0.5, 0, 0, true, true)));

                driver.x().whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCone1));
                driver.y().whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCube2));
                driver.b().whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCone3));
                driver.x()
                        .and(driver.leftTrigger())
                        .whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCoopCone4));
                driver.y()
                        .and(driver.leftTrigger())
                        .whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCoopCube5));
                driver.b()
                        .and(driver.leftTrigger())
                        .whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCoopCone6));
                driver.x()
                        .and(driver.rightTrigger())
                        .whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCone7));
                driver.y()
                        .and(driver.rightTrigger())
                        .whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCube8));
                driver.b()
                        .and(driver.rightTrigger())
                        .whileTrue(swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.redCone9));

                driver.rightBumper().whileTrue(swerve.autoBalance());

                manipulator.start().onTrue(new InstantCommand(() -> leds.handle(0))); //cone
                manipulator.back().onTrue(new InstantCommand(() -> leds.handle(1))); //cube

                manipulator.rightBumper()
                                .whileTrue(claw.autoGrab(true));
                manipulator.leftTrigger().onTrue(claw.release());
                manipulator.rightTrigger().onTrue(claw.grab());

                // manipulator.povUp().whileTrue(superstructure.toState(ScoringStates.TOPCUBE).andThen(claw.release()));
                // manipulator.povLeft().whileTrue(superstructure.toState(ScoringStates.MIDCUBE).andThen(claw.release()));
                // manipulator.y().whileTrue(superstructure.toState(ScoringStates.TOPCONE).andThen(claw.release()));
                // manipulator.x().whileTrue(superstructure.toState(ScoringStates.MIDCONE).andThen(claw.release()));
                // manipulator.povDown().whileTrue(superstructure.toState(ScoringStates.BOT).andThen(claw.release()));
                // manipulator.a().whileTrue(superstructure.toState(ScoringStates.BOT).andThen(claw.release()));
                // manipulator.povRight().whileTrue(superstructure.toState(ScoringStates.SUBSTATION_PICK_UP).andThen(claw.release()));

                // wrist first then wait 0.5 s and then everything else

                // manipulator.povUp().whileTrue(
                //         new ParallelCommandGroup(
                //                 wrist.toAngle(WristK.kTopCubeAngleDegrees), // top cube
                //                 new WaitCommand(0.5)
                //                 .andThen(elevator.toHeight(ElevatorK.kTopCubeHeightM))
                //                 .alongWith(tilt.toAngle(TiltK.kTopCubeAngleDegrees))))
                //         .onFalse(claw.release());

                manipulator.povUp().whileTrue(
                        new ParallelCommandGroup(
                                wrist.toAngle(WristK.kTopCubeAngleDegrees), // top cube
                                new WaitCommand(waitTimeSecondsWrist)
                                .andThen(elevator.toHeight(ElevatorK.kTopCubeHeightM)),
                                new WaitCommand(waitTimeSecondsEle)
                                .andThen(tilt.toAngle(TiltK.kTopCubeAngleDegrees))))
                        .onFalse(claw.release());

                // manipulator.povLeft().whileTrue(
                //         new ParallelCommandGroup(
                //                 wrist.toAngle(WristK.kMidCubeAngleDegrees), // mid cube
                //                 new WaitCommand(0.5)
                //                 .andThen(elevator.toHeight(ElevatorK.kMidCubeHeightM))
                //                 .alongWith(tilt.toAngle(TiltK.kMidCubeAngleDegrees))))
                //         .onFalse(claw.release());

                manipulator.povLeft().whileTrue(
                        new ParallelCommandGroup(
                                wrist.toAngle(WristK.kMidCubeAngleDegrees), // mid cube
                                new WaitCommand(waitTimeSecondsWrist)
                                .andThen(elevator.toHeight(ElevatorK.kMidCubeHeightM)),
                                new WaitCommand(waitTimeSecondsEle)
                                .andThen(tilt.toAngle(TiltK.kMidCubeAngleDegrees))))
                        .onFalse(claw.release());

                manipulator.y().whileTrue(
                        new ParallelCommandGroup(
                                wrist.toAngle(WristK.kTopConeAngleDegrees), // top cone
                                new WaitCommand(waitTimeSecondsWrist)
                                .andThen(elevator.toHeight(ElevatorK.kTopConeHeightM)),
                                new WaitCommand(waitTimeSecondsEle)
                                .andThen(tilt.toAngle(TiltK.kTopConeAngleDegrees))))
                        .onFalse(claw.release());
                        

                manipulator.x().whileTrue(
                        new ParallelCommandGroup(
                                wrist.toAngle(WristK.kTopConeAngleDegrees), // mid cone
                                new WaitCommand(waitTimeSecondsWrist)
                                .andThen(elevator.toHeight(ElevatorK.kTopConeHeightM)),
                                new WaitCommand(waitTimeSecondsEle)
                                .andThen(tilt.toAngle(TiltK.kTopConeAngleDegrees))))
                        .onFalse(claw.release());

                manipulator.povDown().whileTrue(
                        new ParallelCommandGroup(
                                wrist.toAngle(WristK.kBotAngleDegrees), // bottom
                                new WaitCommand(waitTimeSecondsWrist)
                                .andThen(elevator.toHeight(ElevatorK.kBotHeightMeters)),
                                new WaitCommand(waitTimeSecondsEle)
                                .andThen(tilt.toAngle(TiltK.kBotAngleDegrees))))
                        .onFalse(claw.release());
                                
                manipulator.a().whileTrue(
                        new ParallelCommandGroup(
                                wrist.toAngle(WristK.kBotAngleDegrees), // bottom
                                new WaitCommand(waitTimeSecondsWrist)
                                .andThen(elevator.toHeight(ElevatorK.kBotHeightMeters)),
                                new WaitCommand(waitTimeSecondsEle)
                                .andThen(tilt.toAngle(TiltK.kBotAngleDegrees))))
                        .onFalse(claw.release());

                manipulator.povRight().whileTrue(
                        new ParallelCommandGroup(
                                wrist.toAngle(WristK.kSubstationAngleDegrees), // substation
                                new WaitCommand(waitTimeSecondsWrist)
                                .andThen(elevator.toHeight(ElevatorK.kSubstationHeightM)),
                                new WaitCommand(waitTimeSecondsEle)
                                .andThen(tilt.toAngle(TiltK.kSubstationAngleDegrees))))
                        .onFalse(claw.grab());

                manipulator.leftBumper().whileTrue((wrist.toAngle(72)) // to zero
                                // .andThen(new WaitCommand(0.3))
                                .andThen(elevator.toHeight(ElevatorK.kMinHeightMeters -.019)
                                .alongWith(tilt.toAngle(0))));

                                                /*Tuning buttons */
                // manipulator.b().whileTrue(wrist.toAngle(70));
                // manipulator.x().whileTrue(wrist.toAngle(0));
                // manipulator.b().whileTrue(elevator.toHeight(0.3));
                // manipulator.a().whileTrue(tilt.toAngle(15));
                // manipulator.y().whileTrue(tilt.toAngle(0));
                // manipulator.rightBumper().whileTrue(wrist.toAngle(WristK.kMaxAngleDegrees));
        }

        public void mapAutonCommands() {
               AutonChooser.AssignAutonCommand(AutonOption.BACK_OUT, AutonFactory.WaltonPPAuto(swerve, backOut));
               AutonChooser.AssignAutonCommand(AutonOption.CONE, AutonFactory.fullAuto(swerve, oneCone));
               AutonChooser.AssignAutonCommand(AutonOption.CUBE_CONE_1, AutonFactory.WaltonPPAuto(swerve, cubeConeNonBumper));
               AutonChooser.AssignAutonCommand(AutonOption.CUBE_CONE_2, AutonFactory.WaltonPPAuto(swerve, cubeConeBumper));
               AutonChooser.AssignAutonCommand(AutonOption.CONE_RED, AutonFactory.fullAuto(swerve, oneConeRed));
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

        public enum GamePieceMode {
                CONE,
                CUBE
            }
}