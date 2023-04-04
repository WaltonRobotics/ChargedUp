package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auton.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureToState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperState;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.VisionManager;
import frc.robot.auton.AutonChooser.AutonOption;
import frc.robot.auton.Paths.PPPaths;
import frc.robot.auton.Paths.ReferencePoints.ScoringPoints;

import static frc.robot.auton.AutonFactory.autonEventMap;

import java.util.Optional;

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
	public final VisionManager vision = new VisionManager();
	public final SwerveSubsystem swerve = new SwerveSubsystem(autonEventMap, vision);
	public final TiltSubsystem tilt = new TiltSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();
	public final WristSubsystem wrist = new WristSubsystem();
	
	/* Subsystems */	
	public final Superstructure superstructure = new Superstructure(tilt, elevator, wrist, leds);
	public final TheClaw claw = new TheClaw(() -> superstructure.getCurState().claw, ()-> wrist.getDegrees());

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		mapAutonCommands();
		mapAutonEvents();
		// addPathChoices();
		// addAprilTagChoices();
		swerve.setDefaultCommand(
			swerve.teleopDriveCmd(
				() -> driver.getLeftY(),
				() -> driver.getLeftX(),
				() -> -driver.getRightX(),
				() -> false,
				() -> true // openLoop
			)
		);
		elevator.setDefaultCommand(elevator.teleopCmd(() -> -manipulator.getLeftY()));
		tilt.setDefaultCommand(tilt.teleopCmd(() -> manipulator.getRightY()));
		wrist.setDefaultCommand(wrist.teleopCmd(() -> manipulator.getLeftX()));

		// DashboardManager.addTab("TeleSwerve");
		configureButtonBindings();

		// LED triggering
		claw.grabOkTrig.onTrue(leds.grabOk());
		if(DriverStation.isTeleop()){
			claw.grabOkTrig.onTrue(Commands.waitSeconds(.1).andThen(new SuperstructureToState(superstructure, SuperState.SAFE)));
		}
		if(DriverStation.isAutonomous()){
			claw.grabOkTrig.onTrue(Commands.waitSeconds(.1).andThen(new SuperstructureToState(superstructure, SuperState.SAFE).asProxy()));
		}
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
		driver.back().onTrue(swerve.teleOpReset());
		driver.start().onTrue(new InstantCommand(() -> swerve.resetToAbsolute()));
		driver.leftBumper().whileTrue(swerve.nowItsTimeToGetFunky()); // TODO: change reverse condition
		driver.rightBumper().onTrue(swerve.stopWithXCmd());

		driver.x().whileTrue(swerve.autoScore(ScoringPoints.cone1));
		driver.y().whileTrue(swerve.autoScore(ScoringPoints.cube2));
		driver.b().whileTrue(swerve.autoScore(ScoringPoints.cone3));
		driver.x()
			.and(driver.leftTrigger())
			.whileTrue(swerve.autoScore(ScoringPoints.coopCone4));
		driver.y()
			.and(driver.leftTrigger())
			.whileTrue(swerve.autoScore(ScoringPoints.coopCube5));
		driver.b()
			.and(driver.leftTrigger())
			.whileTrue(swerve.autoScore(ScoringPoints.coopCone6));
		driver.x()
			.and(driver.rightTrigger())
			.whileTrue(swerve.autoScore(ScoringPoints.cone7));
		driver.y()
			.and(driver.rightTrigger())
			.whileTrue(swerve.autoScore(ScoringPoints.cube8));
		driver.b()
			.and(driver.rightTrigger())
			.whileTrue(swerve.autoScore(ScoringPoints.cone9));
		

		manipulator.start().toggleOnTrue(Commands.startEnd(leds::setCone, leds::setCube, leds));
		manipulator.leftTrigger().whileTrue(claw.release().repeatedly());
		manipulator.rightTrigger().onTrue(claw.grab());

		manipulator.povUp().onTrue(
			superstructure.cubeTossTop(claw, false));

		manipulator.povLeft().onTrue(
			superstructure.cubeTossMid(claw, false));
				
		manipulator.y().onTrue(
			superstructure.toStateTeleop(SuperState.TOPCONE));

		manipulator.x().onTrue(
			superstructure.toStateTeleop(SuperState.MIDCONE));

		manipulator.a().onTrue(
			superstructure.toStateTeleop(SuperState.GROUND_PICK_UP));
		
		manipulator.b().onTrue(
			superstructure.toStateTeleop(SuperState.EXTENDED_SUBSTATION));

		manipulator.povDown().onTrue(
			superstructure.toStateTeleop(SuperState.GROUND_SCORE));

		manipulator.povRight().onTrue(
			superstructure.toStateTeleop(SuperState.SUBSTATION_PICK_UP));

		manipulator.leftBumper().onTrue(
			superstructure.toStateTeleop(SuperState.SAFE).alongWith(claw.grab()));

		/* Tuning buttons */
		// manipulator.b().whileTrue(wrist.toAngle(70));
		// manipulator.x().whileTrue(wrist.toAngle(0));
		// manipulator.b().whileTrue(elevator.toHeight(0.3));
		// manipulator.a().whileTrue(tilt.toAngle(29));
		// manipulator.y().whileTrue(tilt.toAngle(0));
		// manipulator.rightBumper().whileTrue(wrist.toAngle(WristK.kMaxAngleDegrees));
	}

	public void mapAutonCommands() {
		AutonChooser.SetDefaultAuton(AutonOption.DO_NOTHING);
		AutonChooser.AssignAutonCommand(AutonOption.DO_NOTHING, Commands.none());
		AutonChooser.AssignAutonCommand(AutonOption.ONE_CONE_OUT, 
			AutonFactory.coneBackOut(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.backPark.getInitialHolonomicPose());
		AutonChooser.AssignAutonCommand(AutonOption.CONE_BACK_PARK, 
			AutonFactory.coneBackPark(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.backPark.getInitialHolonomicPose());
		AutonChooser.AssignAutonCommand(AutonOption.CUBE_BACK_PARK, 
			AutonFactory.cubeBackPark(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.backPark.getInitialHolonomicPose());
		AutonChooser.AssignAutonCommand(AutonOption.CONE_ONE_HALF_PARK, 
			AutonFactory.coneOneHalfPark(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.coneOneHalf.get(0).getInitialHolonomicPose());
		AutonChooser.AssignAutonCommand(AutonOption.CUBE_ONE_HALF_PARK, 
			AutonFactory.cubeOneHalfPark(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.cubeOneHalf.get(0).getInitialHolonomicPose());
		AutonChooser.AssignAutonCommand(AutonOption.TWO_ELEMENT, 
			AutonFactory.twoElement(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.twoEle.get(0).getInitialHolonomicPose());
		// AutonChooser.AssignAutonCommand(AutonOption.TWO_ELEMENT_BUMPY, 
		// 	AutonFactory.twoElementBumpy(swerve, superstructure, claw, elevator, tilt, wrist),
		// 	PPPaths.twoEleBumpy.get(0).getInitialHolonomicPose());
		AutonChooser.AssignAutonCommand(AutonOption.TWO_ELEMENT_PARK, 
			AutonFactory.twoElementPark(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.twoEle.get(0).getInitialHolonomicPose());
		AutonChooser.AssignAutonCommand(AutonOption.TWO_ELEMENT_PARK_ALT, 
			AutonFactory.twoElementParkAlt(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.twoEleAlt.get(0).getInitialHolonomicPose());
		AutonChooser.AssignAutonCommand(AutonOption.THREE_ELEMENT, 
			AutonFactory.threeElement(swerve, superstructure, claw, elevator, tilt, wrist),
			PPPaths.twoEle.get(0).getInitialHolonomicPose());
}

	public void mapAutonEvents() {
	}

	public Optional<Pose2d> getAutonomousInitPose() {
		return AutonChooser.GetChosenAutonInitPose();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return AutonChooser.GetChosenAutonCmd();
	}

	public enum GamePieceMode {
		CONE, CUBE
	}
}