package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.auton.Paths.ScoringPoints;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.TiltSubsystem.TiltStates;
import frc.robot.subsystems.WristSubsystem.WristStates;
import static frc.robot.Constants.WristK.*;
import static frc.robot.Constants.ElevatorK.*;

public class Superstructure extends SubsystemBase{
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

    public CommandBase toState(ElevatorStates elevatorState, TiltStates tiltState, WristStates wristState) {
        return runOnce(() -> {
            m_elevator.toState(elevatorState);
            m_tilt.toState(tiltState);
            m_wrist.toState(wristState);
;        });
    }

    public CommandBase toState(ScoringStates state) {
        return toState(state.elevatorHeight, state.elevatorTilt, state.wristTilt);
    }

    public enum ScoringStates {
        MAX(ElevatorStates.MAX, TiltStates.MAX, WristStates.MAX),
        MID(ElevatorStates.MID, TiltStates.MID, WristStates.MID),
        MIN(ElevatorStates.MIN, TiltStates.MIN, WristStates.MIN);
        
        public ElevatorStates elevatorHeight;
        public TiltStates elevatorTilt;
        public WristStates wristTilt;

        private ScoringStates(ElevatorStates elevatorHeight, TiltStates elevatorTilt, WristStates wristTilt) {
            this.elevatorHeight = elevatorHeight;
            this.elevatorTilt = elevatorTilt;
            this.wristTilt = wristTilt;
        }
    }

    /*
     * Dynamically change the max (downwards) angle of the wrist
     * in degrees based on height and tilt of the elevator
     */
    public void limitWristDynamic(){
        double dynamicLimit = kMinAngleDegrees;
        if(m_elevator.getActualHeightRaw() >= kSafeHeight){
            dynamicLimit = kMaxAngleDegrees;
        }
        m_wrist.setMaxDegrees(dynamicLimit);
    }

    // aka autoScore
    // TODO: pass in swerve subsystem, it's not included in this class
    /**
     * @param state high, mid, or low
     * @param place where to score :D (also includes cone/cube mode)
     * @param isBumpy if it is bumpy or not
     * @return command to autoscore
     */
    // public CommandBase win(ScoringStates state, Paths.ScoringPoints.ScoringPlaces place, boolean isBumpy) {
    //     var leds = runOnce(() -> m_leds.handleLED(place.coneOrCube));
    //     var autoScore = runOnce(() -> {
    //         if(isBumpy) {
    //             if(DriverStation.getAlliance().equals(Alliance.Red)) {
    //                 m_swerve.autoScore(PPAutoscoreClass.redBumpy, ScoringPoints.toPathPoint(place.redPt));
    //             }
    //             else {
    //                 m_swerve.autoScore(PPAutoscoreClass.blueBumpy, ScoringPoints.toPathPoint(place.redPt));
    //             }
    //         }
    //         else {
    //             if(DriverStation.getAlliance().equals(Alliance.Red)) {
    //                 m_swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.toPathPoint(place.redPt));
    //             }
    //             else {
    //                 m_swerve.autoScore(PPAutoscoreClass.blueNotBumpy, ScoringPoints.toPathPoint(place.redPt));
    //             }
    //         }
    //     });
    //     var elevatorHeight = runOnce(() -> {
    //         m_elevator.setState(state.elevatorHeight);
    //     });
    //     var finalPos = runOnce(() -> {
    //         m_tilt.setTiltTarget(state.elevatorTilt.angle); // finish later maybe?
    //         m_wrist.toPosition(.5, state.wristTilt);
    //     });
    //     return leds.andThen(autoScore).andThen(elevatorHeight).andThen(finalPos);
    // }
}