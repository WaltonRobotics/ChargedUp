package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.WristK.*;

import frc.robot.Constants;
import frc.robot.Constants.WristK;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkMax m_wristMotor = new CANSparkMax(kWristCANID, MotorType.kBrushless);  // change device number later
  private final SparkMaxAbsoluteEncoder m_absoluteEncoder =  m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
  // private final Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);  // change channel later
  private double wristAngleSetpoint = 0;
  private double wristFFEffort = 0;
  private double wristPDEffort = 0;
  private boolean zeroed;


  private final ProfiledPIDController m_wristController = new ProfiledPIDController(
		WristK.kP, 0, WristK.kD, WristK.kConstraints
	);

  //FFEffort = feedforward; PDEffort = proportional-derivative
  private final GenericEntry nte_wristMotorFFEffort, nte_wristMotorPDEffort, 
                             nte_wristMotorTotalEffort, nte_wristMotorTargetAngle,
                            nte_wristMotorActualAngle, nte_wristMotorTemp;


  /** Creates a new Intake. */
  public WristSubsystem() {

    zeroed = false;

    DashboardManager.addTab(this);


    // m_wristMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

    nte_wristMotorFFEffort = DashboardManager.addTabDial(this, "WristFFEffort", -1, 1);
		nte_wristMotorPDEffort = DashboardManager.addTabDial(this, "WristPDEffort", -1, 1);
		nte_wristMotorTotalEffort = DashboardManager.addTabDial(this, "WristTotalEffort", -1, 1);
		nte_wristMotorTargetAngle = DashboardManager.addTabNumberBar(this, "WristTargetAngle",
			WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
		nte_wristMotorActualAngle = DashboardManager.addTabNumberBar(this, "WristActualAngle",
			WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
        nte_wristMotorTemp = DashboardManager.addTabNumberBar(this,"WristMotorTemp",0,100);
  }

  // public boolean isIntakeOpen() {
  //   return m_intakeSolenoid.get();
  // }

  // public void setIntakeOpen(boolean isOpen) {
  //   m_intakeSolenoid.set(isOpen);
  // }

  public double ticksToDegrees(double ticks) {
    return ticks * (1 / (kAbsEncoderTicksPerRotation / 360));
  }

  public double degreesToTicks(double degrees) {
    return degrees * (kAbsEncoderTicksPerRotation / 360);
  }

  public void setWristToAngle(double speed, double targetAngle) {
    
    while(getWristAngle() != targetAngle) {
      if(getWristAngle() - targetAngle < 0) {
        m_wristMotor.setInverted(true);
        m_wristMotor.set(speed);
      }
      else if(getWristAngle() - targetAngle > 0) {
        m_wristMotor.set(speed);
      }
    }
  }

  public double getWristAngle() {
    return ticksToDegrees(m_absoluteEncoder.getPosition());
  }



  @Override
  public void periodic() {
    updateShuffleBoard();
  }

  public void updateShuffleBoard(){
    if(nte_wristMotorTemp != null)
    nte_wristMotorTotalEffort.setDouble(wristPDEffort+wristFFEffort);
    nte_wristMotorFFEffort.setDouble(wristFFEffort);
    nte_wristMotorPDEffort.setDouble(wristPDEffort);
    nte_wristMotorActualAngle.setDouble(getWristAngle());
    nte_wristMotorTargetAngle.setDouble(wristAngleSetpoint);
    nte_wristMotorTemp.setDouble(m_wristMotor.getMotorTemperature());
  }
}