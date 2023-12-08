package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.WristK.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

import frc.lib.logging.WaltLogger;
import frc.lib.logging.WaltLogger.DoubleLogger;
import frc.robot.Constants.WristK;

public class WristSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(kCANID, MotorType.kBrushless);
  private final SparkMaxAbsoluteEncoder m_absEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
  private double m_targetAngle = kMaxDeg;
  private double m_ffEffort = 0;
  private double m_pdEffort = 0;
  private double m_totalEffort = 0;
  private double m_maxDegrees = kMaxDeg;
  private boolean m_isCoast = false;

  private final DoubleLogger log_wristDegrees = WaltLogger.logDouble(DB_TAB_NAME, "degrees");

  private final PIDController m_holdController = new PIDController(
      kP, 0, kD);

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      WristK.kP, 0, WristK.kD, WristK.kConstraints);

  private final GenericEntry nte_isCoast;

  public WristSubsystem() {
    m_motor.setIdleMode(IdleMode.kBrake); // ANTI-DROOPY
    m_motor.setInverted(true);
    m_motor.setSmartCurrentLimit(kCurrLimit);
    m_absEncoder.setInverted(false);
    m_absEncoder.setPositionConversionFactor(360);
    m_motor.burnFlash();

    m_controller.setTolerance(1.5);
    nte_isCoast = Shuffleboard.getTab(DB_TAB_NAME)
                  .add("wrist coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();
  }

  /*
   * @param coast Whether or not to set motor to coast/brake
   * true for coast, false for brake
   */
  public void setCoast(boolean coast) {
    m_motor.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
  }

  /*
   * @return The actual degree of the wrist (this dynamically changes)
   */
  public double getDegrees() {
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
    m_maxDegrees = MathUtil.clamp(degrees, kMaxDeg, kMinDeg);
  }

  /*
   * @return Whether or not wrist is straight up
   */
  private boolean atMinLimit() {
    return getDegrees() <= kMinDeg;
  }

  /*
   * @return Whether or not wrist is as low as possible
   */
  private boolean atMaxLimit() {
    return getDegrees() >= kMaxDeg;
  }

  private void i_setTarget(double targetAngle) {
    m_targetAngle = MathUtil.clamp(targetAngle, kMinDeg, m_maxDegrees);
  }

  public Command setTarget(double targetAngle) {
    return run(() -> i_setTarget(targetAngle));
  }

  /*
   * Sets the power in velocity or voltage of the wrist motor with soft limits
   * 
   * @param power The velocity/power to set the wrist to
   * 
   * @param voltage Whether or not power should be in velocity (false) or voltage
   * (true)
   */
  private void setPower(double power, boolean voltage) {
    double output = power;
    double dir = Math.signum(power);

    if (atMinLimit() && dir == -1) {
      output = 0;
    } else if (atMaxLimit() && dir == 1) {
      output = 0;
    }
    if (voltage) {
      m_motor.setVoltage(output);
    } else {
      m_motor.set(output);
    }
  }

  /*
   * @return The total effort to reach the setpointAngle
   * 
   * @param setpointAngle The angle to go to
   */
  public double getEffortForTarget(double setpointAngle) {
    m_pdEffort = m_controller.calculate(getDegrees(), setpointAngle);
    var pdSetpoint = m_controller.getSetpoint();
    if (pdSetpoint.velocity != 0) {
      m_ffEffort = kS * Math.signum(m_pdEffort);
    }
    m_totalEffort = m_ffEffort + m_pdEffort;
    return m_pdEffort;
  }

  private double getEffortToHold(double degrees) {
    m_pdEffort = m_holdController.calculate(getDegrees(), degrees);
    m_ffEffort = 0;
    var pdSetpoint = m_holdController.getSetpoint();
    if (pdSetpoint != 0) {
      m_ffEffort = kHoldKs;
    }
    double totalEffort = m_ffEffort + m_pdEffort;
    return totalEffort;
  }
  /*
   * @return Cmd to move the wrist with stick
   * 
   * @param power to apply to wrist motor
   */
  public Command teleopCmd(DoubleSupplier power) {
    return run(() -> {
      double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
      m_targetAngle += powerVal;
      double effort = powerVal == 0 ? getEffortToHold(m_targetAngle) : getEffortForTarget(m_targetAngle);
      setPower(effort, true);
    });
  }

  /*
   * @return Cmd to move wrist to a specified angle
   * 
   * @param angle The angle to move to
   */
  public Command toAngle(DoubleSupplier angle) {
    return run(() -> {
      m_targetAngle = angle.getAsDouble();
      m_pdEffort = m_controller.calculate(getDegrees(), m_targetAngle);
      m_totalEffort = m_ffEffort + m_pdEffort;
      setPower(m_totalEffort, true);
    })
        .until(m_controller::atSetpoint)
        .withName("ToAngle");
  }

  public Command toAngle(double angle) {

    return runOnce(() -> {
      m_controller.reset(getDegrees());
      i_setTarget(angle);
    }).andThen(run(() -> {
      var effort = getDegrees() == angle ? getEffortToHold(angle)
          : MathUtil.clamp(getEffortForTarget(m_targetAngle), -12, 12);
      setPower(effort, true);
    }))
        .until(() -> {
          return m_controller.atGoal();
        })
        .finallyDo((intr) -> {
          m_motor.set(0);
        })
        .withTimeout(1.6)
        .withName("ToAngle");
  }



  public Command toState(WristState state) {
    return toAngle(state.angle);
  }

  @Override
  public void periodic() {
    log_wristDegrees.accept(getDegrees());
    m_isCoast = nte_isCoast.getBoolean(false);
    setCoast(m_isCoast);
  }

  public static enum WristState {
    MAX(kMaxDeg, 0),
    SUBSTATION(kSubstationDeg, 0),
    TOPCONE(kTopConeDeg, 0),
    TOPCUBE(kTopCubeDeg, 1),
    MIDCONE(kMidConeDeg, 0),
    MIDCUBE(kMidCubeDeg, 1),
    PICKUP(kPickupDeg, 0),
    MIN(kMinDeg, 0),
    EXTENDED_SUBSTATION(kExtendedSubstationDeg,0);

    public final double angle;
    public final int isCube;

    private WristState(double angle, int isCube) {
      this.angle = angle;
      this.isCube = isCube;
    }
  }
}