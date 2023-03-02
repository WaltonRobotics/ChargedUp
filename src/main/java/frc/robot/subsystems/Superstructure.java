package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.auton.Paths.ScoringPoints;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.TheClaw.ClawState;
import frc.robot.subsystems.TiltSubsystem.TiltState;
import frc.robot.subsystems.WristSubsystem.WristState;
import static frc.robot.Constants.WristK.*;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ElevatorK.*;

public class Superstructure extends SubsystemBase {
	private final TiltSubsystem m_tilt;
	private final ElevatorSubsystem m_elevator;
	private final WristSubsystem m_wrist;
	private final TheClaw m_claw;

	// State management
	private SuperState m_prevState = SuperState.SAFE;
	private SuperState m_curState = SuperState.SAFE;
	private BooleanSupplier m_wristWait = () -> true;
	private BooleanSupplier m_elevWait = () -> true;
	private BooleanSupplier m_tiltWait = () -> true;
	
	public Superstructure(TiltSubsystem tilt, ElevatorSubsystem elevator, WristSubsystem wrist, TheClaw claw) {
		m_tilt = tilt;
		m_elevator = elevator;
		m_wrist = wrist;
		m_claw = claw;
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
		var stateManageCmd = runOnce(() -> {
			// keep track of SuperState
			m_prevState = m_curState;
			m_curState = state;
			/**
			 * Risky Transitions
			 * from * to TOP*
			 * from * to MID*
			 * from TOP* to *
			 * from MID* to *
			 * 
			 */
			
			if (m_curState == SuperState.TOPCONE || m_curState == SuperState.TOPCUBE) {
				m_elevWait = () -> (m_tilt.getDegrees() >= m_curState.tilt.angle / 2);
				m_wristWait = () -> (m_elevator.getActualHeightMeters() >= m_curState.elev.height);
			}

			if (m_curState == SuperState.MIDCONE || m_curState == SuperState.MIDCUBE) {
				m_elevWait = () -> (m_tilt.getDegrees() >= m_curState.tilt.angle / 2.0);
				m_wristWait = () -> (m_elevator.getActualHeightMeters() >= m_curState.elev.height);
			}

			if (m_curState == SuperState.SAFE && 
				(m_prevState == SuperState.TOPCONE || m_prevState == SuperState.TOPCUBE)) {

			}

			if (m_curState == SuperState.SAFE && 
				(m_prevState == SuperState.TOPCONE || m_prevState == SuperState.TOPCUBE)) {
					m_elevWait = () -> (m_wrist.getDegrees() >= m_curState.wrist.angle);
					m_tiltWait = () -> (m_elevator.getActualHeightMeters() <= m_curState.elev.height);
			}
		});

		var wristCmd = new WaitUntilCommand(m_wristWait).andThen(m_wrist.setTarget(state.wrist.angle));
		var elevCmd = new WaitUntilCommand(m_elevWait).andThen(m_elevator.setTarget(state.elev.height));
		var tiltCmd = new WaitUntilCommand(m_tiltWait).andThen(m_tilt.setTarget(state.tilt.angle));
		var clawCmd = m_claw.getCmdForState(state.claw);

		if (m_curState == SuperState.GROUND_PICK_UP || m_curState == SuperState.SUBSTATION_PICK_UP) {
			clawCmd = m_claw.autoGrab(false);
		}

		var movement = new ParallelCommandGroup(
			wristCmd,
			elevCmd,
			tiltCmd,
			clawCmd
		);

		return stateManageCmd.andThen(movement).withName("ToState-" + m_curState.toString());
	}

	// public CommandBase zeroSuperstructure() {
	//     return m_wrist.toAngle(WristK.kMinAngleDegrees)
	//             .andThen(m_elevator.toHeight(ElevatorK.kMinHeightMeters))
	//             .andThen(m_tilt.toAngle(TiltK.kMinAngleDegrees));
	// }

	public void setTargetsToZero() {
		m_wrist.setTarget(kMaxDeg);
		m_tilt.setTarget(0);
		m_elevator.setTarget(kBotHeightMeters);
	}

/**
 * 	manipulator.leftBumper().onTrue((wrist.toAngle(72)) // to zero
				// .andThen(new WaitCommand(0.3))
				.andThen(elevator.toHeight(ElevatorK.kMinHeightMeters -.019)
				.alongWith(tilt.toAngle(0))));
 */

	public enum SuperState {
		GROUND_PICK_UP(ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.PICKUP, ClawState.AUTO),
		SUBSTATION_PICK_UP(ElevatorState.SUBSTATION, TiltState.SUBSTATION, WristState.SUBSTATION, ClawState.AUTO),
		TOPCONE(ElevatorState.TOPCONE, TiltState.TOPCONE, WristState.TOPCONE, ClawState.IGNORE),
		TOPCUBE(ElevatorState.TOPCUBE, TiltState.TOPCUBE, WristState.TOPCUBE, ClawState.IGNORE),
		MIDCONE(ElevatorState.MIDCONE, TiltState.MIDCONE, WristState.MIDCONE, ClawState.IGNORE),
		MIDCUBE(ElevatorState.MIDCUBE, TiltState.MIDCUBE, WristState.MIDCUBE, ClawState.IGNORE),
		GROUND_SCORE(ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.PICKUP, ClawState.IGNORE),
		SAFE(ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.MAX, ClawState.CLOSE);

		public final ElevatorState elev;
		public final TiltState tilt;
		public final WristState wrist;
		public final ClawState claw;

		private SuperState(ElevatorState elev, TiltState tilt, WristState wrist, ClawState claw) {
			this.elev = elev;
			this.tilt = tilt;
			this.wrist = wrist;
			this.claw = claw;
		}
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