package frc.robot.subsystems.superstructure;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.WaltLogger;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.TiltK;
import frc.robot.Constants.WristK;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.WristSubsystem;
import static frc.robot.Constants.WristK.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ElevatorK.*;

public class Superstructure {
	protected final TiltSubsystem m_tilt;
	protected final ElevatorSubsystem m_elevator;
	protected final WristSubsystem m_wrist;
	protected final LEDSubsystem m_leds;

	// State management
	SuperState m_prevState = SuperState.SAFE;
	private SuperState m_curState = SuperState.SAFE;
	private final DoublePublisher log_autoState = WaltLogger.makeDoublePub("State", "Superstructure/AutonState");
	protected final StringPublisher log_currState = WaltLogger.makeStringPub("State", "Superstructure/CurrState");
	protected final StringPublisher log_prevState = WaltLogger.makeStringPub("State", "Superstructure/PrevState");
	protected final StringPublisher log_stateQuirk = WaltLogger.makeStringPub("State", "Superstructure/StateQuirk");
	
	public Superstructure(TiltSubsystem tilt, ElevatorSubsystem elevator, WristSubsystem wrist, LEDSubsystem leds) {
		m_tilt = tilt;
		m_elevator = elevator;
		m_wrist = wrist;
		// m_claw = claw;
		m_leds = leds;

		log_autoState.accept(-1);
		log_currState.setDefault("UNK");
		log_prevState.setDefault("UNK");
		log_stateQuirk.setDefault("UNK");
	}

	/*
	 * Dynamically change the max (downwards) angle of the wrist
	 * in degrees based on height and tilt of the elevator
	 */
	public void limitWristDynamic() {
		double dynamicLimit = 0;
		if (m_elevator.getActualHeightRaw() >= kSafeHeight) {
			dynamicLimit = kMaxDeg;
		}
		m_wrist.setMaxDegrees(dynamicLimit);
	}

	/*
	 * As soon as elevator is tilted,
	 * change lower limit of elevator
	 */
	public void limitElevatorDynamic() {
		if (!m_tilt.atReverseLimit()) {
			m_elevator.setDynamicLimit(kMinHeightMeters + Units.inchesToMeters(2));
		}
	}

	public CommandBase toStateTeleop(SuperState state) {
		return new SuperstructureToState(this, state, true);
	}

	public CommandBase toStateAuton(SuperState state) {
		return new SuperstructureToState(this, state, false);
	}

	private CommandBase cubeToss(SuperState state, TheClaw claw, boolean auton) {
		BooleanSupplier clawWait = () -> (m_elevator.getActualHeightMeters() >= m_curState.elev.height *.25);
		var toStateCmd = auton ? toStateAuton(state) : toStateTeleop(state);

		return Commands.parallel(
			toStateCmd,
			Commands.waitUntil(clawWait).andThen(claw.release())
		);
	}

	public CommandBase cubeTossMid(TheClaw claw, boolean auton) {
		return cubeToss(SuperState.MIDCUBE, claw, auton);
	}

	public CommandBase cubeTossTop(TheClaw claw, boolean auton) {
		return cubeToss(SuperState.TOPCUBE, claw, auton);
	}

	public void initState() {
		m_prevState = SuperState.SAFE;
		m_curState = SuperState.SAFE;
		m_elevator.setCoast(false);
		m_wrist.setCoast(false);
		m_tilt.setCoast(false);
	}

	public CommandBase smartReset() {
		var tiltCmd = m_tilt.toAngle(TiltK.kBotAngleDegrees);
		var elevCmd = m_elevator.toHeight(ElevatorK.kBotHeightMeters);
		var wristCmd = m_wrist.toAngle(WristK.kMaxDeg);
		return Commands.parallel(
            tiltCmd,	
            wristCmd,
            elevCmd
        );
	}

	protected void updateState(SuperState newState) {
		System.out.println(
			"[SS] upateState - WAS " + m_prevState +
			", FROM " + m_curState +
			" TO " + newState);
		m_prevState = m_curState;
		m_curState = newState;
		log_autoState.accept(m_curState.idx);
	}

	public SuperState getPrevState() {
		return m_prevState;
	}

	public SuperState getCurState() {
		return m_curState;
	}

	public void periodicTelemetry() {
		log_currState.accept(m_curState.toString());
		log_prevState.accept(m_prevState.toString());
	}
}