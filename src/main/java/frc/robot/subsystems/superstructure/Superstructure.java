package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.WristSubsystem;
import static frc.robot.Constants.WristK.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ElevatorK.*;

public class Superstructure extends SubsystemBase {
	protected final TiltSubsystem m_tilt;
	protected final ElevatorSubsystem m_elevator;
	protected final WristSubsystem m_wrist;
	protected final TheClaw m_claw;

	// State management
	private SuperState m_prevState = SuperState.SAFE;
	private SuperState m_curState = SuperState.SAFE;

	public Superstructure(TiltSubsystem tilt, ElevatorSubsystem elevator, WristSubsystem wrist, TheClaw claw) {
		m_tilt = tilt;
		m_elevator = elevator;
		m_wrist = wrist;
		m_claw = claw;

		DashboardManager.addTab(this);
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

	public void reset() {
		m_prevState = SuperState.SAFE;
		m_curState = SuperState.SAFE;
	}

	protected void updateState(SuperState newState) {
		System.out.println(
				"[SS] upateState - WAS " + m_prevState +
						", FROM " + m_curState +
						" TO " + newState);
		m_prevState = m_curState;
		m_curState = newState;
	}

	protected SuperState getPrevState() {
		return m_prevState;
	}

	protected SuperState getCurState() {
		return m_curState;
	}
}