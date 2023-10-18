package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.TheClawK.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class TheClaw extends SubsystemBase {
	private final Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, kTheID);
	private final DigitalInput clawSensor = new DigitalInput(kClawSensor);

	private final Supplier<ClawState> m_autoStateSupplier;
	private final Supplier<Double> m_wristDegSupplier;

	private final Timer m_lastActuationTimer = new Timer();
	private final Timer m_substationDelayTimer = new Timer();
	private boolean isFlapExtended = false;

	private static final double kSensorCheckDelay = 0.3;

	private boolean m_isClosed = false;
	private boolean m_grabOk = false;

	private final Servo m_leftFlap = new Servo(kLeftServo);
	private final Servo m_rightFlap = new Servo(kRightServo);

	public final Trigger sensorTrig = new Trigger(clawSensor::get).negate();
	public final Trigger closedTrig = new Trigger(() -> m_isClosed);
	public final Trigger grabOkTrig = new Trigger(() -> m_grabOk);
	private final Trigger stateAutoGrabTrig;
	private final Trigger substationStateAutoGrabTrig;
	private final Trigger extendedStateAutoGrabTrig;
	private final Trigger safeTrig;
	private final Trigger sensorCheckValidTrig = new Trigger(() -> m_lastActuationTimer.hasElapsed(kSensorCheckDelay));
	private final Trigger substationDelayTrig = new Trigger(() -> m_substationDelayTimer.hasElapsed(kSensorCheckDelay));
	private final Trigger wristAngleTrig;

	public TheClaw(Supplier<ClawState> autoStateSupplier, Supplier<Double> wristDegSupplier) {
		double subsysInitBegin = Timer.getFPGATimestamp();
		System.out.println("[INIT] ClawSubsystem Init Begin");
		m_autoStateSupplier = autoStateSupplier;
		m_lastActuationTimer.restart();
		m_wristDegSupplier = wristDegSupplier;
		m_substationDelayTimer.restart();
		wristAngleTrig = new Trigger(() -> m_wristDegSupplier.get().doubleValue() <= 50);

		// m_rightFlap.setBounds(2.5, 1.5, 1.5, 1.5, 0.5);
		// m_leftFlap.setBounds(2.5, 1.5, 1.5, 1.5, 0.5);

		stateAutoGrabTrig = new Trigger(() -> m_autoStateSupplier.get() == ClawState.AUTO);
		extendedStateAutoGrabTrig = new Trigger(() -> m_autoStateSupplier.get() == ClawState.EXTENDEDAUTO);
		substationStateAutoGrabTrig = new Trigger(() -> m_autoStateSupplier.get() == ClawState.SUBSTATIONAUTO);
		safeTrig = new Trigger(() -> m_autoStateSupplier.get() == ClawState.CLOSE);

		var sensorAutonDebounceTrig = new Trigger(
				new BooleanSupplier() {
					final Debouncer m_debouncer = new Debouncer(0.04);

					@Override
					public boolean getAsBoolean() {
						if (DriverStation.isAutonomous()) {
							return m_debouncer.calculate(sensorTrig.getAsBoolean());
						} else {
							return sensorTrig.getAsBoolean();
						}
					}
				});

		stateAutoGrabTrig.onTrue(
				release()
						.andThen(runOnce(() -> m_grabOk = false)));

		stateAutoGrabTrig
				.and(sensorAutonDebounceTrig)
				.and(sensorCheckValidTrig)
				.onTrue(
						Commands.runOnce(() -> m_grabOk = true)
								.andThen(grab()).withName("internalAutoGrab"));

		extendedStateAutoGrabTrig.onTrue(
				release()
						.alongWith(extendFlaps(true))
						.andThen(runOnce(() -> m_grabOk = false)));

		extendedStateAutoGrabTrig
				.and(sensorAutonDebounceTrig)
				.and(sensorCheckValidTrig)
				.onTrue(
						Commands.runOnce(() -> m_grabOk = true)
								.andThen(grab().alongWith(extendFlaps(false))).withName("internalExtendedAutoGrab"));

		safeTrig.onTrue(
				extendFlaps(false));

		substationStateAutoGrabTrig.onTrue(
				Commands.sequence(
						Commands.runOnce(() -> m_substationDelayTimer.restart()),
						Commands.waitUntil(wristAngleTrig),
						release(),
						Commands.runOnce(() -> m_substationDelayTimer.restart()),
						Commands.runOnce(() -> m_grabOk = false).withName("internalAutoGrabSSReset")));
		substationStateAutoGrabTrig
				.and(sensorAutonDebounceTrig)
				.and(substationDelayTrig)
				.and(closedTrig.negate())
				.onTrue(
						Commands.runOnce(() -> m_grabOk = true)
								.andThen(grab()).withName("internalAutoGrabSSAct"));

		double subsysInitElapsed = Timer.getFPGATimestamp() - subsysInitBegin;
		System.out.println("[INIT] ClawSubsystem Init End: " + subsysInitElapsed + "s");
	}

	private void setClosed(boolean closed) {
		m_lastActuationTimer.restart();
		m_substationDelayTimer.restart();
		claw.set(!closed);
		m_isClosed = closed;
	}

	/**
	 * @return Cmd to release claw
	 */
	public CommandBase release() {
		return runOnce(() -> {
			setClosed(false);
		});
	}

	public CommandBase grab() {
		return runOnce(() -> {
			setClosed(true);
		});
	}

	public CommandBase extendFlaps(boolean extend) {
		return extendFlaps(extend, false);
	}

	// TODO: make it work good (i think it should be continuous)
	// notes for me from drive:
	// with the below code during teleop it extends the flaps during safe and
	// retracts them during ground pickup for some reason, when both of them should
	// be in theory going in the same direction but
	public CommandBase extendFlaps(boolean extend, boolean ignoreState) {
		return Commands.startEnd(() -> {
			boolean shouldMove = ignoreState ? true : extend ? !isFlapExtended : isFlapExtended;
			if (extend && shouldMove) {
				m_rightFlap.setPosition(0);
				m_leftFlap.setPosition(1);
				isFlapExtended = true;
			} else if (!extend && shouldMove) {
				m_rightFlap.setPosition(1);
				m_leftFlap.setPosition(0);
				isFlapExtended = false;
			} else {
				// m_rightFlap.setPosition(extend ? 1 : 0);
				// m_leftFlap.setPosition(extend ? 0 : 1);
			}
		}, () -> {
			m_rightFlap.setPosition(0.5);
			m_leftFlap.setPosition(0.5);
			System.out.println(isFlapExtended);
		}).withTimeout(0.9);
		// return Commands.none();
	}

	public enum ClawState {
		IGNORE,
		OPEN,
		CLOSE,
		AUTO,
		SUBSTATIONAUTO,
		EXTENDEDAUTO
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("CLAW EYE", sensorTrig.getAsBoolean());
	}
}
