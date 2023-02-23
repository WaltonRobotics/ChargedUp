package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.WristK.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

import frc.robot.Constants.WristK;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(kWristCANID, MotorType.kBrushless); // change device number
                                                                                               // later
  private final SparkMaxAbsoluteEncoder m_absoluteEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private double m_targetAngle = 0;
  private double m_ffEffort = 0;
  private double m_pdEffort = 0;
  private double m_totalEffort = 0;
  private double m_maxDegrees = kMaxAngleDegrees;

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      WristK.kP, 0, WristK.kD, WristK.kConstraints);

  // FFEffort = feedforward; PDEffort = proportional-derivative
  private final GenericEntry nte_motorFFEffort = DashboardManager.addTabDial(this, "WristFFEffort", -1, 1);
  private final GenericEntry nte_motorPDEffort = DashboardManager.addTabDial(this, "WristPDEffort", -1, 1);
  private final GenericEntry nte_totalEffort = DashboardManager.addTabDial(this, "WristTotalEffort", -1, 1);
  private final GenericEntry nte_targetAngle = DashboardManager.addTabNumberBar(this, "WristTargetAngle",
      WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
  private final GenericEntry nte_rawAbsEncoder = DashboardManager.addTabDial(this, "RawAbsEnc", 0, 1);
  private final GenericEntry nte_actualAngle = DashboardManager.addTabNumberBar(this, "WristActualAngle",
      WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
  private final GenericEntry nte_coast = DashboardManager.addTabBooleanToggle(this, "Wrist Coast");
  private final GenericEntry nte_motorTemp = DashboardManager.addTabNumberBar(this, "WristMotorTemp", 0, 100);
  private final GenericEntry nte_bottomLimit = DashboardManager.addTabBooleanBox(this, "BotLimit");
  private final GenericEntry nte_topLimit = DashboardManager.addTabBooleanBox(this, "TopLimit");

  /** Creates a new Intake. */
  public WristSubsystem() {

    //ANTI-DROOPY
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(kWristCurrLimit);
    // DashboardManager.addTab(this);
  }

  // public boolean isIntakeOpen() {
  // return m_intakeSolenoid.get();
  // }

  // public void setIntakeOpen(boolean isOpen) {
  // m_intakeSolenoid.set(isOpen);
  // }

  private void setCoast(boolean coast) {
		if (coast) {
			m_motor.setIdleMode(IdleMode.kCoast);
		} else {
			m_motor.setIdleMode(IdleMode.kBrake);
		}
	}

  private double getDegrees() {
    return (m_absoluteEncoder.getPosition() * 360) + kZeroDegOffset;
  }

  public double degreesToTicks(double degrees) {
    return degrees * (kAbsEncoderTicksPerRotation / 360);
  }

  //  TODO: review with grace why this is bad - 3 reasons
  // public void setWristToAngle(double speed, double targetAngle) {

  //   while (getWristAngle() != targetAngle) {
  //     if (getWristAngle() - targetAngle < 0) {
  //       m_wristMotor.setInverted(true);
  //       m_wristMotor.set(speed);
  //     } else if (getWristAngle() - targetAngle > 0) {
  //       m_wristMotor.set(speed);
  //     }
  //   }
  // }

  public double getWristMaxDegrees(){
    return m_maxDegrees;
  }

  public void setWristMaxDegrees(double degrees){
    m_maxDegrees = MathUtil.clamp(degrees, kMaxAngleDegrees, kMinAngleDegrees);
  } 
  private boolean atBottomLimit() {
    return getDegrees() >= m_maxDegrees;
  }

  private boolean atTopLimit() {
    return getDegrees() <= kMinAngleDegrees;
  }

  private void setWristPower(double power) {
    double output = power;
    double dir = Math.signum(power);

    if(atBottomLimit() && dir == 1) {
      output = 0;
      System.out.println("Wrist - BotLimit!!!");
    } else if(atTopLimit() && dir == -1) {
      output = 0;
      System.out.println("Wrist - TopLimit!!!");
    }
    m_motor.set(output);
  }

  public CommandBase teleopWristCmd(DoubleSupplier power){
    return run(() ->{
      double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
      setWristPower(powerVal);
    });
  }

  public CommandBase toAngle(DoubleSupplier angle) {
    return run(()-> {
      m_targetAngle = angle.getAsDouble();
      // m_wristPDEffort = m_wristController.calculate(getDegrees(), m_wristTargetAngle);
      m_totalEffort = m_ffEffort + m_pdEffort;

      setWristPower(m_totalEffort);
    })
    .until(m_controller::atSetpoint)
    .withName("ToAngle");
  }

  public CommandBase toPosition(double speed, WristStates position) {
    switch (position) {
      case MAX:
        // setWristToAngle(speed, MathUtil.clamp(WristStates.MAX.degrees, 0, minValue));
        break;
      case HIGH:
        // setWristToAngle(speed, MathUtil.clamp(WristStates.HIGH.degrees, 0, minValue));
        break;
      case MID:
        // setWristToAngle(speed, MathUtil.clamp(WristStates.MID.degrees, 0, minValue));
        break;
      case MIN:
        // setWristToAngle(speed, MathUtil.clamp(WristStates.MIN.degrees, 0, minValue));
        break;
    }

    // TODO: command-ify
    return Commands.none();
  }

  @Override
  public void periodic() {
    
    // setCoast(nte_coast.get().getBoolean());
    updateShuffleBoard();
  }

  public void updateShuffleBoard() {
    if (nte_motorTemp != null) {
      nte_totalEffort.setDouble(m_totalEffort);
      nte_motorFFEffort.setDouble(m_ffEffort);
      nte_motorPDEffort.setDouble(m_pdEffort);
      nte_actualAngle.setDouble(getDegrees());
      nte_rawAbsEncoder.setDouble(m_absoluteEncoder.getPosition());
      nte_targetAngle.setDouble(m_targetAngle);
      nte_motorTemp.setDouble(m_motor.getMotorTemperature());
      nte_bottomLimit.setBoolean(atBottomLimit());
      nte_topLimit.setBoolean(atTopLimit());
    }
    
    boolean shouldCoast = nte_coast.getBoolean(false);
    m_motor.setIdleMode(shouldCoast ? IdleMode.kCoast : IdleMode.kBrake);
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