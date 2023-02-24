package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public void limitWristDynamic(){
        double dynamicLimit = kMinAngleDegrees;
        if(m_elevator.getElevatorHeight() >= kSafeHeight){
        }
        m_wrist.setWristMinDegrees(dynamicLimit);

    }
}