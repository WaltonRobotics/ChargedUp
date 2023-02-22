package frc.robot.subsystems;

import static frc.robot.auton.AutonFactory.autonEventMap;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.auton.Paths;
import frc.robot.auton.Paths.PPAutoscoreClass;
import frc.robot.auton.Paths.ScoringPoints;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.TiltSubsystem.TiltStates;
import frc.robot.subsystems.WristSubsystem.WristStates;
import frc.robot.vision.AprilTagCamera;

public class Superstructure extends SubsystemBase{
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
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

    public LEDSubsystem getLEDs(){
        return m_leds;
    }

    // public CommandBase AutoScore(){


    // }
    
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

    // aka autoScore
    // to do: make it a command :D
    /**
     * @param state high, mid, or low
     * @param place where to score :D (also includes cone/cube mode)
     * @param isBumpy if it is bumpy or not
     * @return command to autoscore
     */
    public CommandBase win(ScoringStates state, Paths.ScoringPoints.ScoringPlaces place, boolean isBumpy) {
        // System.out.println("YAYAYAYAY :DDD");
        var leds = runOnce(() -> m_leds.handleLED(place.coneOrCube));
        var autoScore = runOnce(() -> {
            if(isBumpy) {
                if(DriverStation.getAlliance().equals(Alliance.Red)) {
                    m_swerve.autoScore(PPAutoscoreClass.redBumpy, ScoringPoints.toPathPoint(place.redPt));
                }
                else {
                    m_swerve.autoScore(PPAutoscoreClass.blueBumpy, ScoringPoints.toPathPoint(place.redPt));
                }
            }
            else {
                if(DriverStation.getAlliance().equals(Alliance.Red)) {
                    m_swerve.autoScore(PPAutoscoreClass.redNotBumpy, ScoringPoints.toPathPoint(place.redPt));
                }
                else {
                    m_swerve.autoScore(PPAutoscoreClass.blueNotBumpy, ScoringPoints.toPathPoint(place.redPt));
                }
            }
        });
        var elevatorHeight = runOnce(() -> {
            m_elevator.setState(state.elevatorHeight);
        });
        var finalPos = runOnce(() -> {
            m_tilt.setTiltTarget(state.elevatorTilt.angle); // finish later maybe?
            m_wrist.toPosition(.5, state.wristTilt);
        });
        return leds.andThen(autoScore).andThen(elevatorHeight).andThen(finalPos);
    }
}