package frc.robot.subsystems;

//TODO: redo tilt limits (possible moving the wrong way)
//TODO: have the teleop command use toAngle (same for other subsystems) and stick inputs add or subtract from that
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.TiltK.*;
import static frc.robot.Constants.TiltK.kMotorCANID;
import static frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

public class TiltSubsystem extends SubsystemBase {
	private final CANSparkMax m_motor = new CANSparkMax(kMotorCANID, MotorType.kBrushless);
	private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(kAbsoluteEncoderPort);
	private final Encoder m_quadratureEncoder = new Encoder(kQuadEncoderA, kQuadEncoderB);
	private final DigitalInput m_homeSwitch = new DigitalInput(kHomeSwitchPort);
	private final Solenoid m_diskBrake = new Solenoid(PneumaticsModuleType.REVPH, kDiskBrakePort);
	private Timer m_timer = new Timer();

	private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, 0, kD, kConstraints);
	private double m_targetAngle = 0;
	private double m_ffEffort = 0;
	private double m_pdEffort = 0;
	private double m_totalEffort = 0;
	private final GenericEntry nte_motorPDEffort = DashboardManager.addTabDial(this, "PDEffort", -1, 1);
	private final GenericEntry nte_motorFFEffort = DashboardManager.addTabDial(this, "FFEffort", -1, 1);
	private final GenericEntry nte_motorTotalEffort = DashboardManager.addTabDial(this, "TotalEffort", -1, 1);
	private final GenericEntry nte_targetAngle = DashboardManager.addTabNumberBar(this, "TargetAngle",
			kMinAngleDegrees, kMaxAngleDegrees);
	private final GenericEntry nte_actualAngle = DashboardManager.addTabNumberBar(this, "ActualAngle", 0, 35);
	private final GenericEntry nte_rawAbsVal = DashboardManager.addTabNumberBar(this, "RawAbs", 0, 1);
	private final GenericEntry nte_coast = DashboardManager.addTabBooleanToggle(this, "coast");
	private final GenericEntry nte_homeSwitch = DashboardManager.addTabBooleanBox(this, "HomeSwitch");
	private final GenericEntry nte_forwardLimit = DashboardManager.addTabBooleanBox(this, "forward limit");

	public TiltSubsystem() {
		m_diskBrake.set(true);
		m_absoluteEncoder.reset();
		m_motor.setIdleMode(IdleMode.kBrake);
		m_motor.setSmartCurrentLimit(kMotorCurrLimit);

		// reset relative encoder on switch activation
		m_quadratureEncoder.setIndexSource(m_homeSwitch);
		// m_absoluteEncoder.setPositionOffset(kAbsZeroDegreeOffset/360.0);
	}

	public CommandBase setTarget(double degrees) {
		return runOnce(() -> i_setTarget(degrees));
	}

	/*
	 * Return true if hitting max degree
	 */
	public boolean atForwardLimit() {
		if (getDegrees() >= kAbsMaxDegree) {
			return true;
		}
		return false;
	}

	/*
	 * Return true if at zero
	 */
	public boolean atReverseLimit() {
		return !m_homeSwitch.get();
	}

	private void i_setTarget(double degrees) {
		m_targetAngle = MathUtil.clamp(degrees, 0, 29);
	}

	private double getEffortForTarget(double angleDegrees) {
		m_pdEffort = m_controller.calculate(getDegrees(), angleDegrees);
		m_ffEffort = 0;
		var pdSetpoint = m_controller.getSetpoint();
		if (pdSetpoint.velocity != 0) {
			m_ffEffort = kS * Math.signum(m_pdEffort);
			// m_ffEffort = kFeedforward.calculate(pdSetpoint.velocity);
		}
		m_totalEffort = m_ffEffort + m_pdEffort;
		return m_pdEffort;
	}

	public double getDegrees() {
		var rawDeg = (m_absoluteEncoder.get() * 360);
		return MathUtil.clamp(rawDeg, 0, kMaxAngleDegrees); // get returns rotations, so rotations * (360 degrees / 1
															// rotation)
	}

	public void disengageBrake() {
		if (m_diskBrake.get()) {
			m_diskBrake.set(false);
		}
	}

	public void engageBrake() {
		if (!m_diskBrake.get()) {
			m_diskBrake.set(true);
		}
	}

	public CommandBase teleopCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			m_targetAngle += powerVal * 1.2;
			double effort = getEffortForTarget(m_targetAngle);	
				setVoltage(effort);
		});
	}

	public void setSpeed(double power) {
		double output;
		double dir = Math.signum(power);
		double powerVal = MathUtil.applyDeadband(power, stickDeadband);
		if ((atForwardLimit() && dir == 1) || (atReverseLimit() && dir == -1)) {
			output = 0;
		} else {
			output = powerVal;
		}
		m_motor.set(output);
	}

	public void setVoltage(double voltage) {
		double output;
		double dir = Math.signum(voltage);
		double powerVal = MathUtil.applyDeadband(voltage, stickDeadband);
		if ((atForwardLimit() && dir == 1) || (atReverseLimit() && dir == -1)) {
			output = 0;
		} else {
			output = powerVal;
		}
		m_motor.setVoltage(output);
	}

	public CommandBase toAngle(DoubleSupplier angle) {
		return run(() -> {
			if (angle.getAsDouble() > getDegrees()) {
				double tempMaxVelocity = kMaxVelocity / 2;
				double tempMaxAcceleration = kMaxAcceleration / 2;

				m_controller.setConstraints(new TrapezoidProfile.Constraints(tempMaxVelocity, tempMaxAcceleration));
			} else {
				m_controller.setConstraints(kConstraints);
			}
			setSpeed(getEffortForTarget(angle.getAsDouble()));
		})
				.until(m_controller::atSetpoint)
				.withName("ToAngle");
	}

	/**
	 * disengageBrake
	 * wait(n)
	 * move().until(atSetpoint)
	 * wait(n1)
	 * engagebrake
	 */
	public CommandBase toAngle(double angle) {
		var setupCmd = runOnce(() -> {
			disengageBrake();
		});

		var moveCmd = runOnce(() -> {
			if (angle > getDegrees()) {
				double tempMaxVelocity = kMaxVelocity / 2;
				double tempMaxAcceleration = kMaxAcceleration / 2;

				m_controller.setConstraints(new TrapezoidProfile.Constraints(tempMaxVelocity, tempMaxAcceleration));
			} else {
				m_controller.setConstraints(kConstraints);
			}
			m_controller.reset(getDegrees());
			i_setTarget(angle);
		}).andThen(run(() -> {
			var effort = MathUtil.clamp(getEffortForTarget(m_targetAngle), -8, 8);
			setVoltage(effort);
		}))
		.finallyDo((intr) -> {
			m_motor.set(0);
		})
		.withName("ToAngle");

		var brakeCmd = runOnce(() -> {
			engageBrake();
		});

		return Commands.sequence(
				setupCmd,
				Commands.waitSeconds(kBeforeBrakeTime),
				moveCmd,
				Commands.waitSeconds(kAfterBrakeTime),
				brakeCmd);
	}

	private void setCoast(boolean coast) {
		if (coast) {
			m_motor.setIdleMode(IdleMode.kCoast);
		} else {
			m_motor.setIdleMode(IdleMode.kBrake);
		}
	}

	@Override
	public void periodic() {
		if (!m_homeSwitch.get()) {
			m_absoluteEncoder.reset();
		}
		setCoast(nte_coast.getBoolean(false));
		updateShuffleBoard();
	}

	public void updateShuffleBoard() {
		// Push telemetry
		nte_actualAngle.setDouble(getDegrees());
		nte_rawAbsVal.setDouble(m_absoluteEncoder.get());
		nte_motorFFEffort.setDouble(m_ffEffort);
		nte_motorPDEffort.setDouble(m_pdEffort);
		nte_motorTotalEffort.setDouble(m_totalEffort);
		nte_targetAngle.setDouble(m_targetAngle);
		nte_homeSwitch.setBoolean(atReverseLimit());
		nte_forwardLimit.setBoolean(atForwardLimit());
	}

	public CommandBase toState(TiltState state) {
		return toAngle(state.angle);
	}

	public enum TiltState {
		UPMOST(kMaxAngleDegrees, 0),
		SUBSTATION(kSubstationAngleDegrees, 0),
		TOPCONE(kTopConeAngleDegrees, 0),
		TOPCUBE(kTopCubeAngleDegrees, 1),
		MIDCONE(kMidConeAngleDegrees, 0),
		MIDCUBE(kMidCubeAngleDegrees, 1),
		BOTTOMMOST(kMinAngleDegrees, 0);

		public final double angle;
		public final int isCube;

		private TiltState(double angle, int isCube) {
			this.angle = angle;
			this.isCube = isCube;
		}
	}
}
