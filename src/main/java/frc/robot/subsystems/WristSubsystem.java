package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.WristK.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

import frc.robot.Constants.WristK;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkMax m_wristMotor = new CANSparkMax(kWristCANID, MotorType.kBrushless); // change device number
                                                                                               // later
  private final SparkMaxAbsoluteEncoder m_absoluteEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private double m_wristTargetAngle = 0;
  private double m_wristFFEffort = 0;
  private double m_wristPDEffort = 0;
  private double m_wristTotalEffort = 0;

  private final ProfiledPIDController m_wristController = new ProfiledPIDController(
      WristK.kP, 0, WristK.kD, WristK.kConstraints);

  // FFEffort = feedforward; PDEffort = proportional-derivative
  private final GenericEntry nte_wristMotorFFEffort, nte_wristMotorPDEffort,
      nte_wristMotorTotalEffort, nte_wristMotorTargetAngle,
      nte_wristMotorActualAngle, nte_wristMotorTemp, nte_coast;

  /** Creates a new Intake. */
  public WristSubsystem() {

    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setSmartCurrentLimit(kWristCurrLimit);
    DashboardManager.addTab(this);
    m_wristMotor.setSoftLimit(SoftLimitDirection.kForward, kMaxAnglePosition);
    //m_wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_wristMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    m_wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // m_wristMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

    nte_wristMotorFFEffort = DashboardManager.addTabDial(this, "WristFFEffort", -1, 1);
    nte_wristMotorPDEffort = DashboardManager.addTabDial(this, "WristPDEffort", -1, 1);
    nte_wristMotorTotalEffort = DashboardManager.addTabDial(this, "WristTotalEffort", -1, 1);
    nte_wristMotorTargetAngle = DashboardManager.addTabNumberBar(this, "WristTargetAngle",
        WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
    nte_wristMotorActualAngle = DashboardManager.addTabNumberBar(this, "WristActualAngle",
        WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
    nte_coast = DashboardManager.addTabBooleanBox(this, "tilt coast");
    nte_wristMotorTemp = DashboardManager.addTabNumberBar(this, "WristMotorTemp", 0, 100);
  }

  // public boolean isIntakeOpen() {
  // return m_intakeSolenoid.get();
  // }

  // public void setIntakeOpen(boolean isOpen) {
  // m_intakeSolenoid.set(isOpen);
  // }

  private void setCoast(boolean coast) {
		if (coast) {
			m_wristMotor.setIdleMode(IdleMode.kCoast);
		} else {
			m_wristMotor.setIdleMode(IdleMode.kBrake);
		}
	}

  public double ticksToDegrees(double ticks) {
    return ticks * (360 / kAbsEncoderTicksPerRotation);
  }

  public double degreesToTicks(double degrees) {
    return degrees * (kAbsEncoderTicksPerRotation / 360);
  }

  public void setWristToAngle(double speed, double targetAngle) {

    while (getWristAngle() != targetAngle) {
      if (getWristAngle() - targetAngle < 0) {
        m_wristMotor.setInverted(true);
        m_wristMotor.set(speed);
      } else if (getWristAngle() - targetAngle > 0) {
        m_wristMotor.set(speed);
      }
    }
  }

  public double getWristAngle() {
    return ticksToDegrees(m_absoluteEncoder.getPosition());
  }

  public CommandBase teleopWristCmd(DoubleSupplier power){
    return run(() ->{
      double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
      double dampener = .1;
      m_wristMotor.set(powerVal*dampener);
    });
  }


  public double minValue = 0; // change later

  public void toPosition(double speed, WristStates position) {
    switch (position) {
      case MAX:
        setWristToAngle(speed, MathUtil.clamp(WristStates.MAX.degrees, 0, minValue));
        break;
      case HIGH:
        setWristToAngle(speed, MathUtil.clamp(WristStates.HIGH.degrees, 0, minValue));
        break;
      case MID:
        setWristToAngle(speed, MathUtil.clamp(WristStates.MID.degrees, 0, minValue));
        break;
      case MIN:
        setWristToAngle(speed, MathUtil.clamp(WristStates.MIN.degrees, 0, minValue));
        break;
    }
  }

  @Override
  public void periodic() {
    m_wristPDEffort = m_wristController.calculate(getWristAngle(), m_wristTargetAngle);
    m_wristTotalEffort = m_wristFFEffort + m_wristPDEffort;
    setCoast(nte_coast.get().getBoolean());
    updateShuffleBoard();
  }

  public void updateShuffleBoard() {
    if (nte_wristMotorTemp != null)
    nte_wristMotorTotalEffort.setDouble(m_wristTotalEffort);
    nte_wristMotorFFEffort.setDouble(m_wristFFEffort);
    nte_wristMotorPDEffort.setDouble(m_wristPDEffort);
    nte_wristMotorActualAngle.setDouble(m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    nte_wristMotorTargetAngle.setDouble(m_wristTargetAngle);
    nte_wristMotorTemp.setDouble(m_wristMotor.getMotorTemperature());
    SmartDashboard.putBoolean("wrist idle mode", nte_coast.get().getBoolean());
  }

  public enum WristStates { // change degrees later
    MAX(0),
    HIGH(30),
    MID(60),
    MIN(115);

    public final double degrees;

    private WristStates(double degrees) {
      this.degrees = degrees;
    }
  }
}