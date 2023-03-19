package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.TheClaw.ClawState;
import frc.robot.subsystems.TiltSubsystem.TiltState;
import frc.robot.subsystems.WristSubsystem.WristState;

public enum SuperState {
    GROUND_PICK_UP(ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.PICKUP, ClawState.OPEN),
    SUBSTATION_PICK_UP(ElevatorState.SUBSTATION, TiltState.SUBSTATION, WristState.SUBSTATION, ClawState.OPEN),
    TOPCONE(ElevatorState.TOPCONE, TiltState.TOPCONE, WristState.TOPCONE, ClawState.IGNORE),
    TOPCUBE(ElevatorState.TOPCUBE, TiltState.TOPCUBE, WristState.TOPCUBE, ClawState.OPEN),
    MIDCONE(ElevatorState.MIDCONE, TiltState.MIDCONE, WristState.MIDCONE, ClawState.IGNORE),
    MIDCUBE(ElevatorState.MIDCUBE, TiltState.MIDCUBE, WristState.MIDCUBE, ClawState.OPEN),
    GROUND_SCORE(ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.PICKUP, ClawState.IGNORE),
    SAFE(ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.MAX, ClawState.CLOSE),
    EXTENDED_SUBSTATION(ElevatorState.EXTENDED_SUBSTATION, TiltState.EXTENDED_SUBSTATION, WristState.EXTENDED_SUBSTATION, ClawState.OPEN);

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