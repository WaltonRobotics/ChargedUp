package frc.robot.subsystems.superstructure;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.logging.WaltLogger;
import frc.lib.logging.WaltLogger.DoubleLogger;
import frc.lib.logging.WaltLogger.StringLogger;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.TiltK;
import frc.robot.Constants.WristK;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.util.function.BooleanSupplier;

public class Superstructure {
	protected final TiltSubsystem m_tilt;
	protected final ElevatorSubsystem m_elevator;
	protected final WristSubsystem m_wrist;
	protected final LEDSubsystem m_leds;

	// State management
	SuperState m_prevState = SuperState.SAFE;
	private SuperState m_curState = SuperState.SAFE;
	private final DoubleLogger log_autoState = WaltLogger.logDouble("Superstructure", "AutonState");
	protected final StringLogger log_currState = WaltLogger.logString("Superstructure", "CurrState");
	protected final StringLogger log_prevState = WaltLogger.logString("Superstructure", "PrevState");
	protected final StringLogger log_stateQuirk = WaltLogger.logString("Superstructure", "StateQuirk");
	
	public Superstructure(TiltSubsystem tilt, ElevatorSubsystem elevator, WristSubsystem wrist, LEDSubsystem leds) {
		m_tilt = tilt;
		m_elevator = elevator;
		m_wrist = wrist;
		m_leds = leds;

		log_autoState.accept(-1.0);
		log_currState.accept("UNK");
		log_prevState.accept("UNK");
		log_stateQuirk.accept("UNK");
	}

	public CommandBase toStateTeleop(SuperState state) {
		return new SuperstructureToState(this, state, true);
	}

	public CommandBase toStateAuton(SuperState state) {
		return new SuperstructureToState(this, state, false);
	}

	private CommandBase cubeToss(SuperState state, TheClaw claw, boolean auton) {
		BooleanSupplier clawWait = () -> (m_elevator.getActualHeightMeters() >= m_curState.elev.height *.2);
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
			"[SS] updateState - WAS " + m_prevState +
			", FROM " + m_curState +
			" TO " + newState);
		m_prevState = m_curState;
		m_curState = newState;
		log_autoState.accept((double)m_curState.idx);
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