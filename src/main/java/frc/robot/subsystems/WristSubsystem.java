package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CTREModuleState;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.WristK.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

import frc.robot.Constants.WristK;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(kCANID, MotorType.kBrushless); // change device number
                                                                                               // later
  private final SparkMaxAbsoluteEncoder m_absEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private double m_targetAngle = 0;
  private double m_ffEffort = 0;
  private double m_pdEffort = 0;
  private double m_totalEffort = 0;
  private double m_minDegrees = kMinAngleDegrees;

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      WristK.kP, 0, WristK.kD, WristK.kConstraints);

  // FFEffort = feedforward; PDEffort = proportional-derivative
  private final GenericEntry nte_motorFFEffort = DashboardManager.addTabDial(this, "WristFFEffort", -15, 15);
  private final GenericEntry nte_motorPDEffort = DashboardManager.addTabDial(this, "WristPDEffort", -15, 15);
  private final GenericEntry nte_totalEffort = DashboardManager.addTabDial(this, "WristTotalEffort", -15, 15);
  private final GenericEntry nte_targetAngle = DashboardManager.addTabNumberBar(this, "WristTargetAngle",
      WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
  private final GenericEntry nte_rawAbsEncoder = DashboardManager.addTabDial(this, "RawAbsEnc", 0, 1);
  private final GenericEntry nte_actualAngle = DashboardManager.addTabNumberBar(this, "WristActualAngle",
      WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
  private final GenericEntry nte_coast = DashboardManager.addTabBooleanToggle(this, "Wrist Coast");
  private final GenericEntry nte_motorTemp = DashboardManager.addTabNumberBar(this, "WristMotorTemp", 0, 100);
  private final GenericEntry nte_bottomLimit = DashboardManager.addTabBooleanBox(this, "BotLimit");
  private final GenericEntry nte_topLimit = DashboardManager.addTabBooleanBox(this, "TopLimit");
  private final GenericEntry nte_stickVoltage = DashboardManager.addTabDial(this, "Stick Voltage", -15, 15);

  /** Creates a new Intake. */
  public WristSubsystem() {

    //ANTI-DROOPY
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);
    m_motor.setSmartCurrentLimit(kCurrLimit);
    m_absEncoder.setInverted(false);
    m_absEncoder.setPositionConversionFactor(360);
    m_motor.burnFlash();
    // DashboardManager.addTab(this);
  }

  // public boolean isIntakeOpen() {
  // return m_intakeSolenoid.get();
  // }

  // public void setIntakeOpen(boolean isOpen) {
  // m_intakeSolenoid.set(isOpen);c
  // }

  private void setCoast(boolean coast) {
		if (coast) {
			m_motor.setIdleMode(IdleMode.kCoast);
		} else {
			m_motor.setIdleMode(IdleMode.kBrake);
		}
	}

  private double getDegrees() {
    var rawRads = Units.degreesToRadians(m_absEncoder.getPosition());
    return Units.radiansToDegrees(MathUtil.angleModulus(rawRads));
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

  public double getMinDegrees(){
    return m_minDegrees;
  }

  public void setMinDegrees(double degrees){
    m_minDegrees = MathUtil.clamp(degrees, kMaxAngleDegrees, kMinAngleDegrees);
  } 
  private boolean atBottomLimit() {
    return getDegrees() <= m_minDegrees;
  }

  private boolean atTopLimit() {
    return getDegrees() >= kMaxAngleDegrees;
  }

  private void setPower(double power, boolean voltage) {
    double output = power;
    double dir = Math.signum(power);

    if(atBottomLimit() && dir == -1) {
      output = 0;
      System.out.println("BotLimit!!!");
    } else if(atTopLimit() && dir == 1) {
      output = 0;
      System.out.println("TopLimit!!!");
    }
    if (voltage) {
      m_motor.setVoltage(output * 12);
    } else {
      m_motor.set(output);
    }
  }

  private double getEffortForAngle(double setpointAngle) {
    var setpointAngleRads = Units.degreesToRadians(setpointAngle);

    m_pdEffort = m_controller.calculate(Units.degreesToRadians(getDegrees()), setpointAngleRads);
    // m_ffEffort = kFeedforward.calculate(setpointAngleRads, m_controller.getGoal().velocity);
    m_totalEffort = m_ffEffort + m_pdEffort;
    return m_pdEffort;
  }

  public CommandBase teleopCmd(DoubleSupplier power){
    return run(() ->{
      var volts = power.getAsDouble() * m_motor.getBusVoltage();
      nte_stickVoltage.setDouble(volts);
      double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
      setPower(powerVal, true);
    });
  }

  // public CommandBase toAngle(DoubleSupplier angle) {
  //   return run(()-> {
  //     m_targetAngle = angle.getAsDouble();
  //     // m_wristPDEffort = m_wristController.calculate(getDegrees(), m_wristTargetAngle);
  //     m_totalEffort = m_ffEffort + m_pdEffort;

  //     setWristPower(m_totalEffort, true);
  //   })
  //   .until(m_controller::atSetpoint)
  //   .withName("ToAngle");
  // }

  public CommandBase toFlat(){
    return run(()->{
      setPower(getEffortForAngle(0), true);
    }).finallyDo((intr) -> setPower(0, false));
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
    nte_totalEffort.setDouble(m_totalEffort);
    nte_motorFFEffort.setDouble(m_ffEffort);
    nte_motorPDEffort.setDouble(m_pdEffort);
    nte_actualAngle.setDouble(getDegrees());
    nte_rawAbsEncoder.setDouble(m_absEncoder.getPosition());
    nte_targetAngle.setDouble(m_targetAngle);
    nte_motorTemp.setDouble(m_motor.getMotorTemperature());
    nte_bottomLimit.setBoolean(atBottomLimit());
    nte_topLimit.setBoolean(atTopLimit());
    
    // boolean shouldCoast = nte_coast.getBoolean(false);
    // m_motor.setIdleMode(shouldCoast ? IdleMode.kCoast : IdleMode.kBrake);
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