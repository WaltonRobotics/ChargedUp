package frc.robot.subsystems.superstructure;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.WaltLogger;
import frc.lib.util.DashboardManager;
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

public class Superstructure extends SubsystemBase {
	protected final TiltSubsystem m_tilt;
	protected final ElevatorSubsystem m_elevator;
	protected final WristSubsystem m_wrist;
	// protected final TheClaw m_claw;
	protected final LEDSubsystem m_leds;

	// State management
	SuperState m_prevState = SuperState.SAFE;
	private SuperState m_curState = SuperState.SAFE;

	private final DoublePublisher log_ssAutoState;
	private final StringPublisher log_currState, log_prevState, log_stateQuirk;
	
	public Superstructure(TiltSubsystem tilt, ElevatorSubsystem elevator, WristSubsystem wrist, LEDSubsystem leds) {
		m_tilt = tilt;
		m_elevator = elevator;
		m_wrist = wrist;
		// m_claw = claw;
		m_leds = leds;

		DashboardManager.addTab(this);
		final String topicPrefix = this.getName();

		log_ssAutoState = WaltLogger.makeDoubleTracePub(topicPrefix + "/AutoState");
		log_ssAutoState.accept(-1);
		log_currState = WaltLogger.makeStringTracePub(topicPrefix + "/CurrentState");
		log_currState.setDefault("UNK");
		log_prevState = WaltLogger.makeStringTracePub(topicPrefix + "/PreviousState");
		log_prevState.setDefault("UNK");
		log_stateQuirk = WaltLogger.makeStringTracePub(topicPrefix + "/StateQuirk");
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

	public CommandBase overrideStates(DoubleSupplier elevPow, DoubleSupplier tiltPow, DoubleSupplier wristPow) {
		return runOnce(() -> {
			CommandScheduler.getInstance().cancel(
				m_elevator.getCurrentCommand(), m_tilt.getCurrentCommand(), m_wrist.getCurrentCommand());
			
			m_elevator.teleopCmd(elevPow);
			m_tilt.teleopCmd(tiltPow);
			m_wrist.teleopCmd(wristPow);
		});
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

	public CommandBase calculateControllers(SuperState targetState){
		return runOnce(()->{
			m_elevator.getEffortForTarget(targetState.elev.height);
			m_tilt.getEffortForTarget(targetState.tilt.angle);
			m_wrist.getEffortForTarget(targetState.wrist.angle);
		});
	}

	protected void updateState(SuperState newState) {
		System.out.println(
			"[SS] upateState - WAS " + m_prevState +
			", FROM " + m_curState +
			" TO " + newState);
		m_prevState = m_curState;
		m_curState = newState;
		log_ssAutoState.accept(m_curState.idx);
	}

	public SuperState getPrevState() {
		return m_prevState;
	}

	public SuperState getCurState() {
		return m_curState;
	}
	
	// public CommandBase releaseClaw() {
	// 	return m_claw.release();
	// }

	// public CommandBase autoSafe() {
	// 	if (m_claw.getClosed()) {
	// 		if (m_curState == SuperState.GROUND_PICK_UP || m_curState == SuperState.SUBSTATION_PICK_UP || m_curState == SuperState.EXTENDED_SUBSTATION) {
	// 			return new SuperstructureToState(this, SuperState.SAFE);
	// 		}
	// 	}
	// 	return Commands.none();
	// }
	
	// public CommandBase score(){
	// }

	@Override
	public void periodic() {
		log_currState.accept(m_curState.toString());
		log_prevState.accept(m_prevState.toString());

	}
}