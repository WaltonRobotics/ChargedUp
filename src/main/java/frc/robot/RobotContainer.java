package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auton.*;
import frc.lib.util.DashboardManager;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.AutoBalance;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperState;
import frc.robot.vision.AprilTagCamera;
import frc.robot.auton.AutonChooser.AutonOption;
import frc.robot.auton.Paths.PPAutoscoreClass;
import frc.robot.auton.Paths.ReferencePoints.ScoringPoints;

import static frc.robot.auton.AutonFactory.autonEventMap;

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

	public final LEDSubsystem leds = new LEDSubsystem();
	public final AprilTagCamera vision = new AprilTagCamera();
	public final SwerveSubsystem swerve = new SwerveSubsystem(autonEventMap, vision);
	public final TiltSubsystem tilt = new TiltSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();
	public final WristSubsystem wrist = new WristSubsystem();
	public final TheClaw claw = new TheClaw();

	/* Subsystems */
	public final Superstructure superstructure = new Superstructure(tilt, elevator, wrist, claw);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		mapAutonCommands();
		// addPathChoices();
		// addAprilTagChoices();
		swerve.setDefaultCommand(
				swerve.teleopDriveCmd(
						() -> driver.getLeftY(),
						() -> driver.getLeftX(),
						() -> -driver.getRightX(),
						driver.leftBumper()::getAsBoolean,
						() -> true // openLoop
				));
		elevator.setDefaultCommand(elevator.teleopCmd(() -> -manipulator.getLeftY()));
		tilt.setDefaultCommand(tilt.teleopCmd(() -> manipulator.getRightY()));
		wrist.setDefaultCommand(wrist.teleopCmd(() -> manipulator.getLeftX()));

		DashboardManager.addTab("TeleSwerve");
		configureButtonBindings();

		// LED triggering
		claw.leftEyeTrig.and(claw.rightEyeTrig).and(claw.openTrig)
			.onTrue(leds.grabOk().until(claw.closedTrig));
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
		driver.back().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
		driver.start().onTrue(new InstantCommand(() -> swerve.resetToAbsolute()));
		driver.leftBumper().whileTrue(new AutoBalance(swerve, true));

		// add back later
		driver.x().whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy,
			ScoringPoints.cone1));
		driver.y().whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy,
			ScoringPoints.cube2));
		driver.b().whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy,
			ScoringPoints.cone3));
		driver.x()
			.and(driver.leftTrigger())
			.whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy,
			ScoringPoints.coopCone4));
		driver.y()
			.and(driver.leftTrigger())
			.whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy,
			ScoringPoints.coopCube5));
		driver.b()
			.and(driver.leftTrigger())
			.whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy,
			ScoringPoints.coopCone6));
		driver.x()
			.and(driver.rightTrigger())
			.whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy, ScoringPoints.cone7));
		driver.y()
			.and(driver.rightTrigger())
			.whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy, ScoringPoints.cube8));
		driver.b()
			.and(driver.rightTrigger())
			.whileTrue(swerve.autoScore(PPAutoscoreClass.notBumpy, ScoringPoints.cone9));
		
		driver.rightTrigger().onTrue(leds.setCube());
		driver.leftTrigger().onTrue(leds.setCone());

		driver.leftBumper().whileTrue(new AutoBalance(swerve, true));

		manipulator.start().onTrue(superstructure.overrideStates(
			() -> -manipulator.getLeftY(), () -> manipulator.getRightY(), () -> manipulator.getLeftX()
		)); 

		manipulator.rightBumper()
				.whileTrue(claw.autoGrab(true));

		manipulator.leftTrigger().onTrue(claw.release());
		manipulator.rightTrigger().onTrue(claw.grab());

		manipulator.povUp().onTrue(
				superstructure.toState(SuperState.TOPCUBE));

		manipulator.povLeft().onTrue(
				superstructure.toState(SuperState.MIDCUBE));

		manipulator.y().onTrue(
				superstructure.toState(SuperState.TOPCONE));

		manipulator.x().onTrue(
				superstructure.toState(SuperState.MIDCONE));

		manipulator.a().onTrue(
				superstructure.toState(SuperState.GROUND_PICK_UP));

		manipulator.povDown().onTrue(
				superstructure.toState(SuperState.GROUND_SCORE));

		manipulator.povRight().onTrue(
				superstructure.toState(SuperState.SUBSTATION_PICK_UP));

		manipulator.leftBumper().onTrue(
				superstructure.toState(SuperState.SAFE));

		/* Tuning buttons */
		// manipulator.b().whileTrue(wrist.toAngle(70));
		// manipulator.x().whileTrue(wrist.toAngle(0));
		// manipulator.b().whileTrue(elevator.toHeight(0.3));
		// manipulator.a().whileTrue(tilt.toAngle(29));
		// manipulator.y().whileTrue(tilt.toAngle(0));
		// manipulator.rightBumper().whileTrue(wrist.toAngle(WristK.kMaxAngleDegrees));
	}

	public void mapAutonCommands() {
		AutonChooser.AssignAutonCommand(AutonOption.ONE_CONE_PARK, AutonFactory.oneConePark(swerve, superstructure, claw));
		AutonChooser.AssignAutonCommand(AutonOption.DROP_CONE_BACK, AutonFactory.oneConeBack(swerve, superstructure, claw));

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

	public enum GamePieceMode {
		CONE, CUBE
	}
}