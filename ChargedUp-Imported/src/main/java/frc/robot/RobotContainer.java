package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auton.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureToState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import frc.robot.subsystems.swerve.NewBalance;
// import frc.robot.subsystems.swerve.ReverseBalance;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperState;
import frc.robot.vision.VisionManager;
import frc.robot.auton.AutonChooser.AutonOption;
// import frc.robot.auton.Paths.PPPaths;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

// import static frc.robot.auton.AutonFactory.autonEventMap;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

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
	private static final double maxSpeed = 6; // 6 meters per second desired top speed
	private static final double maxAngularRate = Math.PI; // Half a rotation per second max angular velocity (change if
															// too slow)

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController driver = new CommandXboxController(0); // My driver
	private final CommandXboxController manipulator = new CommandXboxController(1);

	public static final LEDSubsystem leds = new LEDSubsystem();
	private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
	public final VisionManager vision = new VisionManager();
	// public final SwerveSubsystem swerve = new SwerveSubsystem(vision);
	public final TiltSubsystem tilt = new TiltSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();
	public final WristSubsystem wrist = new WristSubsystem();

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
																		// driving in open loop
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(maxSpeed);

	/* Subsystems */
	public final Superstructure superstructure = new Superstructure(tilt, elevator, wrist, leds);
	public final TheClaw claw = new TheClaw(() -> superstructure.getCurState().claw, () -> wrist.getDegrees());

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		mapAutonCommands();
		mapAutonEvents();
		// swerve.setDefaultCommand(
		// swerve.teleopDriveCmd(
		// () -> driver.getLeftY(),
		// () -> driver.getLeftX(),
		// () -> -driver.getRightX(),
		// () -> false,
		// () -> true // openLoop
		// ));
		elevator.setDefaultCommand(elevator.teleopCmd(() -> -manipulator.getLeftY()));
		tilt.setDefaultCommand(tilt.teleopCmd(() -> manipulator.getRightY()));
		wrist.setDefaultCommand(wrist.teleopCmd(() -> manipulator.getLeftX()));

		configureButtonBindings();

		// LED triggering
		claw.grabOkTrig.onTrue(leds.grabOk());
		if (DriverStation.isTeleop()) {
			claw.grabOkTrig.onTrue(
					Commands.waitSeconds(.1).andThen(new SuperstructureToState(superstructure, SuperState.SAFE)));
		}
		if (DriverStation.isAutonomous()) {
			claw.grabOkTrig.onTrue(Commands.waitSeconds(.1)
					.andThen(new SuperstructureToState(superstructure, SuperState.SAFE).asProxy()));
		}

		// NewBalance.m_balanceTrig.onTrue(leds.setBalanced().withTimeout(2));
		// ReverseBalance.m_reverseBalanceTrig.onTrue(leds.setBalanced().withTimeout(2));
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
		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * maxSpeed) // Drive forward with
																									// negative Y
																									// (forward)
						.withVelocityY(-driver.getLeftX() * maxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-driver.getRightX() * maxAngularRate) // Drive counterclockwise with
																					// negative X (right)
				));

		driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
		driver.b().whileTrue(drivetrain
				.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

		// reset the field-centric heading on left bumper press
		driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		drivetrain.registerTelemetry(logger::telemeterize);
		// driver.back().onTrue(swerve.teleOpReset());
		// driver.start().onTrue(new InstantCommand(() -> swerve.resetToAbsolute()));
		// driver.leftBumper().whileTrue(swerve.reverseReverse()); // TODO: change
		// reverse condition
		// driver.rightBumper().onTrue(swerve.stopWithXCmd());

		/* Manipulator Buttons */
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

		manipulator.rightBumper().onTrue(
				superstructure.toStateTeleop(SuperState.EXTENDED_GROUND_PICK_UP));

		manipulator.start().onTrue(
				claw.extendFlaps(false));
		manipulator.back().onTrue(
				claw.extendFlaps(true));

		manipulator.povDown().onTrue(
				superstructure.toStateTeleop(SuperState.GROUND_SCORE));

		manipulator.povRight().onTrue(
				superstructure.toStateTeleop(SuperState.SUBSTATION_PICK_UP));

		manipulator.leftBumper().onTrue(
				superstructure.toStateTeleop(SuperState.SAFE).alongWith(claw.grab()));

	}

	public void mapAutonCommands() {
		AutonChooser.SetDefaultAuton(AutonOption.DO_NOTHING);
		AutonChooser.AssignAutonCommand(AutonOption.DO_NOTHING, Commands.none());
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
}