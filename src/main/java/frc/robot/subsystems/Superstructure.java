package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorK;
import frc.robot.Constants.TiltK;
import frc.robot.Constants.WristK;
// import frc.robot.auton.Paths.ScoringPoints;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.TiltSubsystem.TiltStates;
import frc.robot.subsystems.WristSubsystem.WristStates;
import static frc.robot.Constants.WristK.*;

import static frc.robot.Constants.ElevatorK.*;

public class Superstructure extends SubsystemBase {
    private final TiltSubsystem m_tilt;
    private final ElevatorSubsystem m_elevator;
    private final WristSubsystem m_wrist;
    private final TheClaw m_claw;

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
            dynamicLimit = kMaxAngleDegrees;
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

    public CommandBase toState(ScoringStates state) {
        return m_elevator.toHeight(state.elevatorHeight.height)
                .alongWith(m_tilt.toAngle(state.elevatorTilt.angle))
                .andThen(m_wrist.toAngle(state.wristAngle.angle));
    }

    public CommandBase zero() {
        return m_wrist.toAngle(WristK.kMinAngleDegrees)
                .andThen(m_elevator.toHeight(ElevatorK.kMinHeightMeters))
                .andThen(m_tilt.toAngle(TiltK.kMinAngleDegrees));
    }

    public enum ScoringStates {
        GROUND_PICK_UP(ElevatorStates.MIN, TiltStates.MAX, WristStates.MAX),
        SUBSTATION_PICK_UP(ElevatorStates.SUBSTATION, TiltStates.SUBSTATION, WristStates.SUBSTATION),
        TOPCONE(ElevatorStates.TOPCONE, TiltStates.TOPCONE, WristStates.TOPCONE),
        TOPCUBE(ElevatorStates.TOPCUBE, TiltStates.TOPCUBE, WristStates.TOPCUBE),
        MIDCONE(ElevatorStates.MIDCONE, TiltStates.MIDCONE, WristStates.MIDCONE),
        MIDCUBE(ElevatorStates.MIDCUBE, TiltStates.MIDCUBE, WristStates.MIDCUBE),
        BOT(ElevatorStates.BOT, TiltStates.BOT, WristStates.BOT);

        public ElevatorStates elevatorHeight;
        public TiltStates elevatorTilt;
        public WristStates wristAngle;

        private ScoringStates(ElevatorStates elevatorHeight, TiltStates elevatorTilt, WristStates wristTilt) {
            this.elevatorHeight = elevatorHeight;
            this.elevatorTilt = elevatorTilt;
            this.wristAngle = wristTilt;
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