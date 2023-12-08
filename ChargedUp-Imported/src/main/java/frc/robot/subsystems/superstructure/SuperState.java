package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.TheClaw.ClawState;
import frc.robot.subsystems.TiltSubsystem.TiltState;
import frc.robot.subsystems.WristSubsystem.WristState;

public enum SuperState {
        SAFE(0,
                        ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.MAX, ClawState.CLOSE),
        GROUND_PICK_UP(1,
                        ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.PICKUP, ClawState.AUTO),
        EXTENDED_GROUND_PICK_UP(1,
                        ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.PICKUP, ClawState.EXTENDEDAUTO),
        SUBSTATION_PICK_UP(2,
                        ElevatorState.SUBSTATION, TiltState.SUBSTATION, WristState.SUBSTATION,
                        ClawState.SUBSTATIONAUTO),
        TOPCONE(3,
                        ElevatorState.TOPCONE, TiltState.TOPCONE, WristState.TOPCONE, ClawState.IGNORE),
        FASTTOPCONE(3,
                        ElevatorState.TOPCONE, TiltState.TOPCONE, WristState.TOPCONE, ClawState.IGNORE),
        TOPCUBE(4,
                        ElevatorState.TOPCUBE, TiltState.TOPCUBE, WristState.TOPCUBE, ClawState.OPEN),
        MIDCONE(5,
                        ElevatorState.MIDCONE, TiltState.MIDCONE, WristState.MIDCONE, ClawState.IGNORE),
        MIDCUBE(6,
                        ElevatorState.MIDCUBE, TiltState.MIDCUBE, WristState.MIDCUBE, ClawState.OPEN),
        GROUND_SCORE(7,
                        ElevatorState.MIN, TiltState.BOTTOMMOST, WristState.PICKUP, ClawState.IGNORE),
        EXTENDED_SUBSTATION(8,
                        ElevatorState.EXTENDED_SUBSTATION, TiltState.EXTENDED_SUBSTATION,
                        WristState.EXTENDED_SUBSTATION,
                        ClawState.AUTO);

        public final int idx;
        public final ElevatorState elev;
        public final TiltState tilt;
        public final WristState wrist;
        public final ClawState claw;

        private SuperState(int idx, ElevatorState elev, TiltState tilt, WristState wrist, ClawState claw) {
                this.idx = idx;
                this.elev = elev;
                this.tilt = tilt;
                this.wrist = wrist;
                this.claw = claw;
        }
}