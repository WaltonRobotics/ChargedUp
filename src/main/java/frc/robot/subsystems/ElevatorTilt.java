package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import  static frc.robot.Constants.ElevatorTiltK.*;

public class ElevatorTilt extends SubsystemBase {
    
    // Motors
    private final WPI_TalonFX m_tiltMotor = new WPI_TalonFX(kCANID); // change IDs later

    // Sensors
    private final DutyCycleEncoder m_tiltAbsEnc = new DutyCycleEncoder(kAngleAbsPort);
	private final Encoder m_tiltRelEnc = new Encoder(kAngleRelAPort, kAngleRelBPort);
    
    // Simulation
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
        kMotor, kGearRatio, kJKgMetersSquared,
        kArmLengthMeters, kMinAngleRads, kMaxAngleRads, 
        kArmMassKg, true, null);

    public ElevatorTilt() {

    }
}
