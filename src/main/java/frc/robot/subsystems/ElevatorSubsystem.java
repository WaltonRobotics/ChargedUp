package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorK.kConstraints;
import static frc.robot.Constants.ElevatorK.kD;
import static frc.robot.Constants.ElevatorK.kP;
import static frc.robot.Constants.ElevatorK.kLeftCANID;
import static frc.robot.Constants.ElevatorK.kRightCANID;
import static frc.robot.Constants.ElevatorK.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.DashboardManager;
import frc.robot.CTREConfigs;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;
public class ElevatorSubsystem extends SubsystemBase {
	private final WPI_TalonFX m_left = new WPI_TalonFX(kLeftCANID, canbus);
	private final WPI_TalonFX m_right = new WPI_TalonFX(kRightCANID, canbus);
	private final DigitalInput m_lowerLimit = new DigitalInput(kLowerLimitSwitchPort);

	private final ProfiledPIDController m_controller = new ProfiledPIDController(
			kP, 0, kD, kConstraints);

	private final PIDController m_holdController = new PIDController(
		kPHold, 0, kDHold);

	private double m_targetHeight = 0;
	private double m_dynamicLowLimit = kMinHeightMeters;
	private double m_pdEffort = 0;
	private double m_ffEffort = 0;
	private double m_totalEffort = 0;

	private double m_holdPdEffort = 0;
	private double m_holdFfEffort = 0;
	// private boolean m_isCoast = false;

	private final GenericEntry nte_ffEffort = DashboardManager.addTabDial(this, "FF Effort", -1, 1);
	private final GenericEntry nte_pdEffort = DashboardManager.addTabDial(this, "PD Effort", -1, 1);
	private final GenericEntry nte_totalEffort = DashboardManager.addTabDial(this, "Total Effort", -1, 1);
	private final GenericEntry nte_targetHeight = DashboardManager.addTabNumberBar(this, "Target Height Meters",
			kMinHeightMeters, kMaxHeightMeters);
	private final GenericEntry nte_actualHeight = DashboardManager.addTabNumberBar(this, "Actual Height Meters",
			kMinHeightMeters, kMaxHeightMeters);
	private final GenericEntry nte_actualHeightRaw = DashboardManager.addTabNumberBar(this, "Actual Height Raw", 0,
			10000);
	private final GenericEntry nte_coast = DashboardManager.addTabBooleanToggle(this, "Is Coast");
	private final GenericEntry nte_atLowerLimit = DashboardManager.addTabBooleanBox(this, "At Lower Limit");
	private final GenericEntry nte_pdVelo = DashboardManager.addTabDial(this, "PD Velo", -100, 100);
	private final GenericEntry nte_actualVelo = DashboardManager.addTabNumberBar(this, "ActualVelo Mps",
			-10, 10);

	public ElevatorSubsystem() {
		DashboardManager.addTab(this);
		m_left.configFactoryDefault();
		m_left.configAllSettings(CTREConfigs.Get().leftConfig);

		m_right.configFactoryDefault();
		m_right.configAllSettings(CTREConfigs.Get().rightConfig);

		m_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_right.configVoltageCompSaturation(kVoltageCompSaturationVolts);
		//m_right.enableVoltageCompensation(true);

		m_right.setNeutralMode(NeutralMode.Brake);
		m_left.setNeutralMode(NeutralMode.Brake);

		m_left.follow(m_right);
		m_left.setInverted(TalonFXInvertType.OpposeMaster);
		m_controller.setTolerance(.02);
	}

	/*
	 * Returns the actual height in raw encoder ticks
	 */
	public double getActualHeightRaw() {
		return m_right.getSelectedSensorPosition(0);
	}

	/**
	 * @return Whether or not elevator is at dynamic lower limit
	 */
	public boolean isAtDynamicLimit() {
		return getActualHeightMeters() <= m_dynamicLowLimit;
	}

	/**
	 * Set the new dynamic lower limit of elevator
	 * 
	 * @param heightLimit The new limit in meters
	 */
	public void setDynamicLimit(double heightLimit) {
		m_dynamicLowLimit = heightLimit;
	}

	/**
	 * @return Whether or not elevator is fully retracted to sensor
	 */
	public boolean isFullyRetracted() {
		return !m_lowerLimit.get();
	}

	/*
	 * @return The current height of elevator in meters
	 */
	public double getActualHeightMeters() {
		var falconPos = m_right.getSelectedSensorPosition();
		var meters = Conversions.falconToMeters(
				falconPos, kDrumCircumferenceMeters, kGearRatio);
		return meters;// + kElevatorHeightOffset;
	}

	private double getActualVelocityMps() {
		var falconVelo = m_right.getSelectedSensorVelocity();
		var mps = Conversions.falconToMPS(falconVelo, kDrumCircumferenceMeters, kGearRatio);
		return mps;
	}

	/**
	 * @return A cmd to move the elevator via stick
	 * sets elevator to target height if no input
	 */
	public CommandBase teleOpCmd(DoubleSupplier power) {
		return run(() -> {
			// double fromBottom = (getActualHeightMeters() - kMinHeightMeters) / kMaxHeightMeters;
			// double fromTop = (kMaxHeightMeters - getActualHeightMeters()) / kMaxHeightMeters;
			double dir = Math.signum(power.getAsDouble());
			double output = 0;
			
			if (isFullyRetracted() && dir == -1) {
				output = 0;
			} else {
				output = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			}
			// if (fromBottom <= 0.1 || fromTop <= 0.1) {
			// 	output *= 1 - m_dampener;
			// }

			m_targetHeight += output*.05;
			double effort = getEffortForTarget(m_targetHeight);
			m_right.setVoltage(effort);
		})
				.withName("TeleManual");
	}

	/*
	 * Set target height in meters
	 * Does not allow impossible heights
	 */
	private void i_setTarget(double meters) {
		m_targetHeight = MathUtil.clamp(meters, m_dynamicLowLimit, kMaxHeightMeters);
	}

	/*
	 * Sets both elevator motors to coast/brake
	 */
	private void setCoast(boolean coast) {
		m_left.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
		m_right.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
	}

	/*
	 * Command to change target height
	 */
	public CommandBase setTarget(double meters) {
		return runOnce(() -> {
			i_setTarget(meters);
		});
	}

	/**
	 * @return target height in meters
	 */
	private double getTargetHeightMeters() {
		return m_targetHeight;
	}

	/**
	 * @param heightMeters The target height to reach in meters
	 * 
	 * @return The total effort (ff & pd) required to reach target height
	 */
	private double getEffortForTarget(double heightMeters) {
		m_pdEffort = m_controller.calculate(getActualHeightMeters(), heightMeters);
		m_ffEffort = 0;
		var pdSetpoint = m_controller.getSetpoint();
		if (pdSetpoint.velocity != 0) {
			m_ffEffort = kFeedforward.calculate(pdSetpoint.velocity);
		}
		double totalEffort = m_ffEffort + m_pdEffort;
		nte_ffEffort.setDouble(m_ffEffort);
		nte_pdEffort.setDouble(m_pdEffort);
		nte_totalEffort.setDouble(totalEffort);
		nte_pdVelo.setDouble(pdSetpoint.velocity);
		nte_actualVelo.setDouble(getActualVelocityMps());
		return totalEffort;
	}

	private double getEffortToHold(double heightMeters) {
		m_holdPdEffort = m_holdController.calculate(getActualHeightMeters(), heightMeters);
		m_holdFfEffort = 0;
		var pdSetpoint = m_holdController.getSetpoint();
		if (pdSetpoint != 0) {
			m_ffEffort = kFeedforward.calculate(pdSetpoint);
		}
		double totalEffort = m_holdFfEffort + m_holdPdEffort;
		return totalEffort;
	}

	/**
	 * @return Cmd to move the elevator to specified height w/ pd & ff
	 * @param heightMeters The height to move to
	 */
	public CommandBase toHeight(double heightMeters) {
		// if (Math.abs(getActualHeightMeters() - heightMeters) <= 0.02) {
		// 	return Commands.none().until(() -> Math.abs(getActualHeightMeters() - m_targetHeight) > 0.02).andThen(toHeight(m_targetHeight));
		// }
		
		return runOnce(() -> {
			m_controller.reset(getActualHeightMeters());
			i_setTarget(heightMeters);
		})
				.andThen(run(() -> {
					var effort = 
					MathUtil.clamp(getEffortForTarget(m_targetHeight), -kVoltageCompSaturationVolts,
							kVoltageCompSaturationVolts);
					
					m_right.set(ControlMode.PercentOutput, effort / kVoltageCompSaturationVolts);
				}))
				.until(() -> m_controller.atGoal())
				.finallyDo((intr) -> {
					m_right.set(ControlMode.PercentOutput, 0);
				})
				.withName("AutoToHeight");
	}

	public void holdHeight(){
			var holdEffort = 
					MathUtil.clamp(getEffortToHold(m_targetHeight), -kVoltageCompSaturationVolts,
							kVoltageCompSaturationVolts);
			m_right.set(ControlMode.PercentOutput, holdEffort / kVoltageCompSaturationVolts);
	}

	public enum ElevatorState {
		MAX(kMaxHeightMeters),
		SUBSTATION(kSubstationHeightM),
		TOPCONE(kTopConeHeightM),
		TOPCUBE(kTopCubeHeightM),
		MIDCONE(kMidConeHeightM),
		MIDCUBE(kMidCubeHeightM),
		MIN(kMinHeightMeters);

		public double height;

		ElevatorState(double height) {
			this.height = height;
		}
	}

	@Override
	public void periodic() {
		if (!m_lowerLimit.get()) {
			m_right.setSelectedSensorPosition(0);
		}
		updateShuffleBoard();
		setCoast(nte_coast.getBoolean(false));
	}

	/*
	 * Updates nte values
	 */
	public void updateShuffleBoard() {
		nte_actualHeightRaw.setDouble(getActualHeightRaw());
		nte_actualHeight.setDouble(getActualHeightMeters());
		nte_ffEffort.setDouble(m_ffEffort);
		nte_pdEffort.setDouble(m_pdEffort);
		nte_totalEffort.setDouble(m_totalEffort);
		nte_targetHeight.setDouble(getTargetHeightMeters());
		nte_atLowerLimit.setBoolean(isFullyRetracted());
	}
}
