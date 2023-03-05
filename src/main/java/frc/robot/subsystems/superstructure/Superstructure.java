package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TheClaw;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.WristSubsystem;
import static frc.robot.Constants.WristK.*;

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

	// aka autoScore
	// TODO: pass in swerve subsystem, it's not included in this class
	/**
	 * @param state   high, mid, or low
	 * @param place   where to score :D (also includes cone/cube mode)
	 * @param isBumpy if it is bumpy or not
	 * @return command to autoscore
	 */
	// public CommandBase win(ScoringStates state, Paths.ScoringPoints.ScoringPlaces
	// place, boolean isBumpy) {
	// var leds = runOnce(() -> m_leds.handleLED(place.coneOrCube));
	// var autoScore = runOnce(() -> {
	// if(isBumpy) {
	// if(DriverStation.getAlliance().equals(Alliance.Red)) {
	// m_swerve.autoScore(PPAutoscoreClass.redBumpy,
	// ScoringPoints.toPathPoint(place.redPt));
	// }
	// else {
	// m_swerve.autoScore(PPAutoscoreClass.blueBumpy,
	// ScoringPoints.toPathPoint(place.redPt));
	// }
	// }
	// else {
	// if(DriverStation.getAlliance().equals(Alliance.Red)) {
	// m_swerve.autoScore(PPAutoscoreClass.redNotBumpy,
	// ScoringPoints.toPathPoint(place.redPt));
	// }
	// else {
	// m_swerve.autoScore(PPAutoscoreClass.blueNotBumpy,
	// ScoringPoints.toPathPoint(place.redPt));
	// }
	// }
	// });
	// var elevatorHeight = runOnce(() -> {
	// m_elevator.setState(state.elevatorHeight);
	// });
	// var finalPos = runOnce(() -> {
	// m_tilt.setTiltTarget(state.elevatorTilt.angle); // finish later maybe?
	// m_wrist.toPosition(.5, state.wristTilt);
	// });
	// return leds.andThen(autoScore).andThen(elevatorHeight).andThen(finalPos);
	// }
}