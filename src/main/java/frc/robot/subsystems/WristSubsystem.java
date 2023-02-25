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
  private double m_maxDegrees = kMaxAngleDegrees;
  private boolean m_isCoast = false;

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      WristK.kP, 0, WristK.kD, WristK.kConstraints);

  // FFEffort = feedforward; PDEffort = proportional-derivative
  private final GenericEntry nte_motorFFEffort = DashboardManager.addTabDial(this, "FF Effort", -15, 15);
  private final GenericEntry nte_motorPDEffort = DashboardManager.addTabDial(this, "PD Effort", -15, 15);
  private final GenericEntry nte_totalEffort = DashboardManager.addTabDial(this, "Total Effort", -15, 15);
  private final GenericEntry nte_targetAngle = DashboardManager.addTabNumberBar(this, "Target Angle",
      WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
  private final GenericEntry nte_rawAbsEncoder = DashboardManager.addTabDial(this, "Raw Abs Encoder", 0, 1);
  private final GenericEntry nte_actualAngle = DashboardManager.addTabNumberBar(this, "Actual Angle",
      WristK.kMinAngleDegrees, WristK.kMaxAngleDegrees);
  private final GenericEntry nte_coast = DashboardManager.addTabBooleanToggle(this, "Is Coast");
  private final GenericEntry nte_motorTemp = DashboardManager.addTabNumberBar(this, "Motor Temp", 0, 100);
  private final GenericEntry nte_minLimit = DashboardManager.addTabBooleanBox(this, "At Bot Limit");
  private final GenericEntry nte_maxLimit = DashboardManager.addTabBooleanBox(this, "At Top Limit");
  private final GenericEntry nte_stickVoltage = DashboardManager.addTabDial(this, "Stick Voltage", -15, 15);

  public WristSubsystem() {
    m_motor.setIdleMode(IdleMode.kBrake); // ANTI-DROOPY
    m_motor.setInverted(true);
    m_motor.setSmartCurrentLimit(kCurrLimit);
    m_absEncoder.setInverted(false);
    m_absEncoder.setPositionConversionFactor(360);
    m_motor.burnFlash();
  }

  /*
   * @param coast Whether or not to set motor to coast/brake
   * true for coast, false for brake
   */
  private void setCoast(boolean coast) {
    if (coast) {
      m_motor.setIdleMode(IdleMode.kCoast);
      m_isCoast = true;
    } else {
      m_motor.setIdleMode(IdleMode.kBrake);
      m_isCoast = false;
    }
  }

  /*
   * @return The actual degree of the wrist
   */
  private double getDegrees() {
    var rawRads = Units.degreesToRadians(m_absEncoder.getPosition());
    return Units.radiansToDegrees(MathUtil.angleModulus(rawRads));
  }

  /*
   * @return The maximum degree the wrist can turn currently
   */
  public double getMaxDegrees() {
    return m_maxDegrees;
  }

  /*
   * Sets dynamic max wrist angle in degrees clamped between max and min
   * 
   * @param degrees The max degree to set to
   */
  public void setMaxDegrees(double degrees) {
    m_maxDegrees = MathUtil.clamp(degrees, kMaxAngleDegrees, kMinAngleDegrees);
  }

  /*
   * @return Whether or not wrist is straight up
   */
  private boolean atMinLimit() {
    return getDegrees() <= kMinAngleDegrees;
  }

  /*
   * @return Whether or not wrist is as low as possible
   */
  private boolean atMaxLimit() {
    return getDegrees() >= m_maxDegrees;
  }

  /*
   * Sets the power in velocity or voltage of the wrist motor with soft limits
   * @param power The velocity/power to set the wrist to
   * @param voltage Whether or not power should be in velocity (false) or voltage (true)
   */
  private void setPower(double power, boolean voltage) {
    double output = power;
    double dir = Math.signum(power);

    if (atMinLimit() && dir == -1) {
      output = 0;
      System.out.println("BotLimit!!!");
    } else if (atMinLimit() && dir == 1) {
      output = 0;
      System.out.println("TopLimit!!!");
    }
    if (voltage) {
      m_motor.setVoltage(output * 12);
    } else {
      m_motor.set(output);
    }
  }

  /*
   * @return The total effort to reach the setpointAngle
   * @param setpointAngle The angle to go to
   */
  private double getEffortForAngle(double setpointAngle) {
    var setpointAngleRads = Units.degreesToRadians(setpointAngle);

    m_pdEffort = m_controller.calculate(Units.degreesToRadians(getDegrees()), setpointAngleRads);
    // m_ffEffort = kFeedforward.calculate(setpointAngleRads,
    // m_controller.getGoal().velocity);
    m_totalEffort = m_ffEffort + m_pdEffort;
    return m_pdEffort;
  }

  /*
   * @return Cmd to move the wrist with stick
   * @param power to apply to wrist motor
   */
  public CommandBase teleopCmd(DoubleSupplier power) {
    return run(() -> {
      var volts = power.getAsDouble() * m_motor.getBusVoltage();
      nte_stickVoltage.setDouble(volts);
      double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
      setPower(powerVal, true);
    });
  }

  /*
   * @return Cmd to move wrist to a specified angle
   * @param angle The angle to move to
   */
  public CommandBase toAngle(DoubleSupplier angle) {
    return run(() -> {
      m_targetAngle = angle.getAsDouble();
      m_pdEffort = m_controller.calculate(getDegrees(), m_targetAngle);
      m_totalEffort = m_ffEffort + m_pdEffort;
      setPower(m_totalEffort, true);
    })
        .until(m_controller::atSetpoint)
        .withName("ToAngle");
  }

  /*
   * @return A Cmd to move wrist to horizontal flat
   */
  public CommandBase toFlat() {
    return run(() -> {
      setPower(getEffortForAngle(0), true);
    }).finallyDo((intr) -> setPower(0, false));
  }

  public CommandBase toPosition(double speed, WristStates position) {
    switch (position) {
      case MAX:
        // setWristToAngle(speed, MathUtil.clamp(WristStates.MAX.degrees, 0, minValue));
        break;
      case HIGH:
        // setWristToAngle(speed, MathUtil.clamp(WristStates.HIGH.degrees, 0,
        // minValue));
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
    updateShuffleBoard();
    setCoast(m_isCoast);
  }

  /*
   * Updates nte values
   */
  public void updateShuffleBoard() {
    nte_totalEffort.setDouble(m_totalEffort);
    nte_motorFFEffort.setDouble(m_ffEffort);
    nte_motorPDEffort.setDouble(m_pdEffort);
    nte_actualAngle.setDouble(getDegrees());
    nte_rawAbsEncoder.setDouble(m_absEncoder.getPosition());
    nte_targetAngle.setDouble(m_targetAngle);
    nte_motorTemp.setDouble(m_motor.getMotorTemperature());
    nte_minLimit.setBoolean(atMinLimit());
    nte_maxLimit.setBoolean(atMaxLimit());
    nte_coast.setBoolean(m_isCoast);
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