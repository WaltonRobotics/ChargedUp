package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.logging.WaltLogger;
import frc.lib.logging.WaltLogger.DoubleLogger;

import static frc.robot.Constants.TiltK.*;
import static frc.robot.Constants.TiltK.kMotorCANID;
import static frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

public class TiltSubsystem extends SubsystemBase {
	private final CANSparkMax m_motor = new CANSparkMax(kMotorCANID, MotorType.kBrushless);
	private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(kAbsoluteEncoderPort);
	private final DigitalInput m_homeSwitch = new DigitalInput(kHomeSwitchPort);
	private final Solenoid m_diskBrake = new Solenoid(PneumaticsModuleType.REVPH, kDiskBrakePort);

	private boolean m_isCoast = false;

	private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, 0, kD, kConstraints);
	private final PIDController m_holdController = new PIDController(kPHold, 0, kDHold);
	private double m_targetAngle = 0;
	private static double m_pdEffort = 0;
	private double m_holdPdEffort = 0;
	private double m_holdFfEffort = 0;

	private final DoubleLogger log_actualAngle = WaltLogger.logDouble(DB_TAB_NAME, "ActualAngle");
	private final DoubleLogger log_rawAbsVal = WaltLogger.logDouble(DB_TAB_NAME, "RawAbs");

	public final Trigger m_homeSwitchTrigger = new Trigger(m_homeSwitch::get).negate();
	private final GenericEntry nte_isCoast;

	public static double nte_pdEffort = m_pdEffort;
	
	public TiltSubsystem() {
		double subsysInitBegin = Timer.getFPGATimestamp();
		System.out.println("[INIT] TiltSubsystem Init Begin");
		m_absoluteEncoder.reset();
		m_motor.setIdleMode(IdleMode.kBrake);
		m_motor.setSmartCurrentLimit(kMotorCurrLimit);

		m_homeSwitchTrigger.onTrue(resetEncoder());
		m_holdController.setTolerance(.75);

		double subsysInitElapsed = Timer.getFPGATimestamp() - subsysInitBegin;
		System.out.println("[INIT] TiltSubsystem Init End: " + subsysInitElapsed + "s");

		nte_isCoast = Shuffleboard.getTab(DB_TAB_NAME)
                  .add("tilt coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();
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

	public boolean getHomeSwitch(){
		return m_homeSwitchTrigger.getAsBoolean();
	}

	/*
	 * Return true if at zero
	 */
	public boolean atReverseLimit() {
		return m_homeSwitchTrigger.getAsBoolean();
	}

	private void i_setTarget(double degrees) {
		m_targetAngle = MathUtil.clamp(degrees, 0, kAbsMaxDegree);
	}

	public double getEffortForTarget(double angleDegrees) {
		m_pdEffort = m_controller.calculate(getDegrees(), angleDegrees);
		return m_pdEffort;
	}

	private double getEffortToHold(double degrees) {
		m_holdPdEffort = m_holdController.calculate(getDegrees(), degrees);
		m_holdFfEffort = 0;
		var pdSetpoint = m_holdController.getSetpoint();
		if (pdSetpoint != 0) {
			m_holdFfEffort = kHoldKs;
		}
		double totalEffort = m_holdFfEffort + m_holdPdEffort;
		return totalEffort;
	}

	public CommandBase holdAngle() {
		return run(()->{
			var holdEffort = 
					MathUtil.clamp(getEffortToHold(m_targetAngle), -kVoltageCompSaturationVolts,
							kVoltageCompSaturationVolts);
			setVoltage(holdEffort / kVoltageCompSaturationVolts);
		})
		.withName("Hold Angle");
	}

	public double getDegrees() {
		var rawDeg = ((m_absoluteEncoder.get()) * 360);
		return MathUtil.clamp(rawDeg, 0, kAbsMaxDegree); // get returns rotations, so rotations * (360 degrees / 1
															// rotation)
	}

	public CommandBase resetEncoder(){
		return Commands.runOnce(() -> m_absoluteEncoder.reset()).ignoringDisable(true);
	}

	public CommandBase teleopCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			m_targetAngle += powerVal * 1.2;
			double effort = getEffortForTarget(m_targetAngle);
			double holdEffort = getEffortToHold(m_targetAngle);
			
			if(powerVal > 0){
				setVoltage(effort);
			} else {
				powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
				setVoltage(holdEffort);
			}
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

	public CommandBase autoHome() {
		return new DeferredCommand(() -> {
			if (atReverseLimit()) {
				return Commands.runOnce(() -> m_absoluteEncoder.reset());
			} else {
				return Commands.sequence(
					startEnd(() -> {
						setVoltage(-2);
					}, () -> {
						setVoltage(0);
					}).until(m_homeSwitchTrigger)
					.andThen(atReverseLimit() ? new InstantCommand(() -> m_absoluteEncoder.reset()) : Commands.none())
				);
			}
		}, this);
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
			if (angle > getDegrees()) {
				double tempMaxVelocity = kMaxVelocityForward;
				double tempMaxAcceleration = kMaxAccelerationForward;
				
				m_controller.setConstraints(new TrapezoidProfile.Constraints(tempMaxVelocity, tempMaxAcceleration));
			} else {
				m_controller.setConstraints(kConstraints);
			}
			m_controller.reset(getDegrees());
			i_setTarget(angle);
		});

		var moveCmd = run(() -> {
			var effort = MathUtil.clamp(getEffortForTarget(m_targetAngle), -12, 12);
			setVoltage(effort);
		})
		.until(() -> {
			return m_controller.atGoal();
		})
		.finallyDo((intr) -> {
			m_motor.set(0);
		})
		.withTimeout(1.6)
		.withName("ToAngle");

		return Commands.sequence(
			setupCmd,
			moveCmd
		);
	}

	public void setCoast(boolean coast) {
		if (!DriverStation.isEnabled()) {
			m_diskBrake.set(!coast);
		}
		m_motor.setIdleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
	}

	@Override
	public void periodic() {
		updateShuffleBoard();
		setCoast(m_isCoast);
	}

	public void updateShuffleBoard() {
		// Push telemetry
		log_actualAngle.accept(getDegrees());
		log_rawAbsVal.accept(m_absoluteEncoder.get());
		m_isCoast = nte_isCoast.getBoolean(false);
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
		BOTTOMMOST(kMinAngleDegrees, 0),
		EXTENDED_SUBSTATION(kExtendedSubstationAngleDegrees,0);

		public final double angle;
		public final int isCube;

		private TiltState(double angle, int isCube) {
			this.angle = angle;
			this.isCube = isCube;
		}
	}
}
