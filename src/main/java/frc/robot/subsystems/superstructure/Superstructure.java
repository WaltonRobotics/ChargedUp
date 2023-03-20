package frc.robot.subsystems.superstructure;
//TODO: have autograb based on beam break not seeing anything then seeing things.
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.TiltK;
import frc.robot.Constants.WristK;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.TheClaw.ClawState;

import static frc.robot.Constants.WristK.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ElevatorK.*;

public class Superstructure extends SubsystemBase {
	protected final TiltSubsystem m_tilt;
	protected final ElevatorSubsystem m_elevator;
	protected final WristSubsystem m_wrist;
	protected final TheClaw m_claw;

	// State management
	SuperState m_prevState = SuperState.SAFE;
	private SuperState m_curState = SuperState.SAFE;
	private final GenericEntry nte_currState = DashboardManager.addTabItem(this, "CurrState", "UNK");
	private final GenericEntry nte_prevState = DashboardManager.addTabItem(this, "PrevState", "UNK");
	protected final GenericEntry nte_stateQuirk = DashboardManager.addTabItem(this, "StateQuirk", "UNK");
	
	public Superstructure(TiltSubsystem tilt, ElevatorSubsystem elevator, WristSubsystem wrist, TheClaw claw) {
		m_tilt = tilt;
		m_elevator = elevator;
		m_wrist = wrist;
		m_claw = claw;

		DashboardManager.addTab(this);
		
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

	public CommandBase toState(SuperState state) {
		return new SuperstructureToState(this, state);
	}

	public CommandBase autoReset(){
		return new SuperstructureToState(this, SuperState.SAFE).withTimeout(.5);
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
		var tiltCmd = /* m_tilt.getDegrees() < 2 ? Commands.none() : */ m_tilt.toAngle(TiltK.kBotAngleDegrees);
		var elevCmd = m_elevator.getActualHeightMeters() < .05 ? Commands.none() : m_elevator.toHeight(ElevatorK.kBotHeightMeters);
		var wristCmd = Math.abs(m_wrist.getDegrees() - WristK.kMaxDeg) < 2 ? Commands.none() : m_wrist.toAngle(WristK.kMaxDeg);
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
	}

	public SuperState getPrevState() {
		return m_prevState;
	}

	public SuperState getCurState() {
		return m_curState;
	}
	
	public CommandBase releaseClaw() {
		var clawCmd = m_claw.release();

		// if (getCurState() != SuperState.SAFE && getCurState() != SuperState.GROUND_PICK_UP && getCurState() != SuperState.SUBSTATION_PICK_UP) {
		// 	return clawCmd.andThen(new SuperstructureToState(this, SuperState.SAFE));
		// }

		return clawCmd.andThen(Commands.waitSeconds(1));
	}

	public CommandBase autoSafe() {
		if (m_claw.getState()) {
			if (getCurState() == SuperState.GROUND_PICK_UP || getCurState() == SuperState.SUBSTATION_PICK_UP || getCurState() == SuperState.EXTENDED_SUBSTATION) {
				return new SuperstructureToState(this, SuperState.SAFE);
			}
		}
		return Commands.none();
	}

	@Override
	public void periodic() {
		nte_currState.setString(m_curState.toString());
		nte_prevState.setString(m_prevState.toString());
	}
}