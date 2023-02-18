package frc.robot.subsystems;

import static frc.robot.auton.AutonFactory.autonEventMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.AprilTagCamera;

public class Superstructure extends SubsystemBase{
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final AprilTagCamera m_apriltagCamera = new AprilTagCamera();
    private final SwerveSubsystem m_swerve = new SwerveSubsystem(autonEventMap, m_apriltagCamera);
    private final TheClaw m_claw = new TheClaw();
    private final TiltSubsystem m_tilt = new TiltSubsystem();
    private final WristSubsystem m_wrist = new WristSubsystem();

    public Superstructure(){
    }   
    
    public SwerveSubsystem getSwerve(){
        return m_swerve;
    }
    

    public ElevatorSubsystem getElevator(){
        return m_elevator;
    }

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
}