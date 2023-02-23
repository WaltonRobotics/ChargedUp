package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.WristK.*;

import frc.robot.Constants.WristK;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(kCANID, MotorType.kBrushless); // change device number
                                                                                               // later
  private final SparkMaxAbsoluteEncoder m_absoluteEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private double m_targetAngle = 0;
  private double m_FFEffort = 0;
  private double m_PDEffort = 0;
  private double m_totalEffort = 0;

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      WristK.kP, 0, WristK.kD, WristK.kConstraints);

  // FFEffort = feedforward; PDEffort = proportional-derivative
  private final GenericEntry nte_wristMotorFFEffort, nte_wristMotorPDEffort,
      nte_wristMotorTotalEffort, nte_wristMotorTargetAngle,
      nte_wristMotorActualAngle, nte_wristMotorTemp;

  /** Creates a new Intake. */
  public WristSubsystem() {

    m_motor.setSmartCurrentLimit(kCurrLimit);
    DashboardManager.addTab(this);

    // m_wristMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

    nte_wristMotorFFEffort = DashboardManager.addTabDial(this, "WristFFEffort", -1, 1);
    nte_wristMotorPDEffort = DashboardManager.addTabDial(this, "WristPDEffort", -1, 1);
    nte_wristMotorTotalEffort = DashboardManager.addTabDial(this, "WristTotalEffort", -1, 1);
    nte_wristMotorTargetAngle = DashboardManager.addTabNumberBar(this, "WristTargetAngle",
        WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
    nte_wristMotorActualAngle = DashboardManager.addTabNumberBar(this, "WristActualAngle",
        WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
    nte_wristMotorTemp = DashboardManager.addTabNumberBar(this, "WristMotorTemp", 0, 100);
  }

  // public boolean isIntakeOpen() {
  // return m_intakeSolenoid.get();
  // }

  // public void setIntakeOpen(boolean isOpen) {
  // m_intakeSolenoid.set(isOpen);
  // }

  public double ticksToDegrees(double ticks) {
    return ticks * (1 / (kAbsEncoderTicksPerRotation / 360));
  }

  public double degreesToTicks(double degrees) {
    return degrees * (kAbsEncoderTicksPerRotation / 360);
  }

  public void setToAngle(double speed, double targetAngle) {

    while (getAngle() != targetAngle) {
      if (getAngle() - targetAngle < 0) {
        m_motor.setInverted(true);
        m_motor.set(speed);
      } else if (getAngle() - targetAngle > 0) {
        m_motor.set(speed);
      }
    }
  }

  public double getAngle() {
    return ticksToDegrees(m_absoluteEncoder.getPosition());
  }

  public double minValue = 0; // change later

  public void toPosition(double speed, States position) {
    switch (position) {
      case MAX:
        setToAngle(speed, MathUtil.clamp(States.MAX.degrees, 0, minValue));
        break;
      case HIGH:
        setToAngle(speed, MathUtil.clamp(States.HIGH.degrees, 0, minValue));
        break;
      case MID:
        setToAngle(speed, MathUtil.clamp(States.MID.degrees, 0, minValue));
        break;
      case MIN:
        setToAngle(speed, MathUtil.clamp(States.MIN.degrees, 0, minValue));
        break;
    }
  }

  @Override
  public void periodic() {
    m_PDEffort = m_controller.calculate(getAngle(), m_targetAngle);
    m_totalEffort = m_FFEffort + m_PDEffort;
    updateShuffleBoard();
  }

  public void updateShuffleBoard() {
    if (nte_wristMotorTemp != null)
    nte_wristMotorTotalEffort.setDouble(m_totalEffort);
    nte_wristMotorFFEffort.setDouble(m_FFEffort);
    nte_wristMotorPDEffort.setDouble(m_PDEffort);
    nte_wristMotorActualAngle.setDouble(m_absoluteEncoder.getPosition());
    nte_wristMotorTargetAngle.setDouble(m_targetAngle);
    nte_wristMotorTemp.setDouble(m_motor.getMotorTemperature());
  }

  public enum States { // change degrees later
    MAX(0),
    HIGH(30),
    MID(60),
    MIN(115);

    public final double degrees;

    private States(double degrees) {
      this.degrees = degrees;
    }
  }
}