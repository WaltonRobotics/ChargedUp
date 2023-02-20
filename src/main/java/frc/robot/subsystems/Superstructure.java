package frc.robot.subsystems;

import static frc.robot.auton.AutonFactory.autonEventMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.TiltSubsystem.TiltStates;
import frc.robot.subsystems.WristSubsystem.WristStates;
import frc.robot.vision.AprilTagCamera;

public class Superstructure extends SubsystemBase{
    //private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final AprilTagCamera m_apriltagCamera = new AprilTagCamera();
    private final SwerveSubsystem m_swerve = new SwerveSubsystem(autonEventMap, m_apriltagCamera);
    private final TheClaw m_claw = new TheClaw();
    private final TiltSubsystem m_tilt = new TiltSubsystem();
    private final WristSubsystem m_wrist = new WristSubsystem();
    private final LEDSubsystem m_leds = new LEDSubsystem();

    public Superstructure(){
    }   
    
    public SwerveSubsystem getSwerve(){
        return m_swerve;
    }
    

    // public ElevatorSubsystem getElevator(){
    //     return m_elevator;
    // }

    public TiltSubsystem getTilt(){
        return m_tilt;
    }

    public AprilTagCamera getCamera(){
        return m_apriltagCamera;
    }

    public TheClaw getClaw(){
        return m_claw;
    }

    public WristSubsystem getWrist(){
        return m_wrist;
    }

    public LEDSubsystem getLEDs(){
        return m_leds;
    }

    // public enum ScoringStates {
    //     MAX(ElevatorStates.MAX, TiltStates.MAX, WristStates.MAX),
    //     MID(ElevatorStates.MID, TiltStates.MID, WristStates.MID),
    //     MIN(ElevatorStates.MIN, TiltStates.MIN, WristStates.MIN);
        
    //     public ElevatorStates elevatorHeight;
    //     public TiltStates elevatorTilt;
    //     public WristStates wristTilt;

    //     private ScoringStates(ElevatorStates elevatorHeight, TiltStates elevatorTilt, WristStates wristTilt) {
    //         this.elevatorHeight = elevatorHeight;
    //         this.elevatorTilt = elevatorTilt;
    //         this.wristTilt = wristTilt;
    //     }
    // }
}