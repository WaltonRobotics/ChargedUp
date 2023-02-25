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

	private double m_targetHeight = 0;
	private double m_pdEffort = 0;
	private double m_ffEffort = 0;
	private double m_totalEffort = 0;
	private boolean m_isCoast = false;

	private final GenericEntry nte_ffEffort = DashboardManager.addTabDial(this, "FF Effort", -1, 1);
	private final GenericEntry nte_pdEffort = DashboardManager.addTabDial(this, "PD Effort", -1, 1);
	private final GenericEntry nte_totalEffort = DashboardManager.addTabDial(this, "Total Effort", -1, 1);
	private final GenericEntry nte_targetHeight = DashboardManager.addTabNumberBar(this, "Target Height Meters",
			kMinHeightMeters, kMaxHeightMeters);
	private final GenericEntry nte_actualHeight = DashboardManager.addTabNumberBar(this, "Actual Height Meters",
		kMinHeightMeters, kMaxHeightMeters);
	private final GenericEntry nte_actualHeightRaw = DashboardManager.addTabNumberBar(this, "Actual Height Raw",0,10000);
	private final GenericEntry nte_coast = DashboardManager.addTabBooleanToggle(this, "Is Coast");
	private final GenericEntry nte_atLowerLimit = DashboardManager.addTabBooleanBox(this, "At Lower Limit");

	public ElevatorSubsystem() {
		m_left.configFactoryDefault();
        m_left.configAllSettings(CTREConfigs.Get().leftConfig);

		m_right.configFactoryDefault();
        m_right.configAllSettings(CTREConfigs.Get().rightConfig);

		m_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

		m_right.setNeutralMode(NeutralMode.Brake);
		m_left.setNeutralMode(NeutralMode.Brake);

		m_left.follow(m_right);
		m_left.setInverted(TalonFXInvertType.OpposeMaster);
	}

	/*
	 * Returns the actual height in raw encoder ticks
	 */
	public double getActualHeightRaw() {
		return m_right.getSelectedSensorPosition(0);
	}

	/*
	 * @return Whether or not elevator is fully retracted
	 */
	public boolean isAtLowerLimit() {
		return !m_lowerLimit.get();
	}

	/*
	 * @return The current height of elevator in meters
	 */
	private double getActualHeightMeters() {
		var falconPos = m_right.getSelectedSensorPosition();
		var meters = Conversions.falconToMeters(
				falconPos, kDrumCircumferenceMeters, kGearRatio);
		return meters + kElevatorHeightOffset;
	}

	/*
	 * @return A cmd to move the elevator via stick
	 */
	public CommandBase teleOpCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			m_right.set(ControlMode.PercentOutput, powerVal);
		});
	}

	/*
	 * Set target height in meters
	 * Does not allow impossible heights
	 */
	private void i_setTarget(double meters) {
		m_targetHeight = MathUtil.clamp(meters, kMinHeightMeters, kMaxHeightMeters);
	}

	/*
	 * Sets both elevator motors to coast/brake
	 */
	private void setCoast(boolean coast) {
		if(coast){
			m_left.setNeutralMode(NeutralMode.Coast);
			m_right.setNeutralMode(NeutralMode.Coast);
			m_isCoast = true;
		}
		else{
			m_left.setNeutralMode(NeutralMode.Brake);
			m_right.setNeutralMode(NeutralMode.Brake);
			m_isCoast = false;
		}
	}

	/*
	 * Command to change target height
	 */
	public CommandBase setTarget(double meters) {
		return runOnce(() -> {
			i_setTarget(meters);
		});
	}

	/*
	 * @return target height in meters
	 */
	private double getTargetHeightMeters() {
		return Conversions.MetersToFalcon(m_targetHeight, kDrumCircumferenceMeters, kGearRatio);
	}

	/* @param heightMeters The target height to reach in meters
	 * @return The total effort (ff & pd) required to reach target height
	 */
	private double getEffortForTarget(double heightMeters) {
		double pdEffort = m_controller.calculate(getActualHeightMeters(), heightMeters);

		double ffEffort = 0;

		if (m_controller.getSetpoint().velocity != 0) {
			ffEffort = kFeedforward.calculate(m_controller.getSetpoint().velocity);
		}

		double totalEffort = ffEffort + pdEffort;

		nte_ffEffort.setDouble(ffEffort);
		nte_pdEffort.setDouble(pdEffort);
		nte_totalEffort.setDouble(totalEffort);

		return totalEffort;
	}

	public CommandBase toHeight(double heightMeters) {
		return run(()-> {
			m_right.set(getEffortForTarget(heightMeters));
		}).until(() -> m_controller.atGoal())
		.finallyDo((intr)-> {
			m_right.set(ControlMode.PercentOutput, 0);
		});
	}

	public CommandBase toState(ElevatorStates state) {
		return toHeight(state.height);
	}

	public enum ElevatorStates {
		MAX(kMaxHeightMeters),
		MID(kMaxHeightMeters / 2),
		MIN(kMinHeightMeters);

		public double height;

		ElevatorStates(double height) {
			this.height = height;
		}
	}

	@Override
	public void periodic() {
		if(!m_lowerLimit.get()) {
			m_right.setSelectedSensorPosition(0);
		}
		updateShuffleBoard();
		setCoast(m_isCoast);
	}

	public void updateShuffleBoard(){
		nte_actualHeightRaw.setDouble(getActualHeightRaw());
		nte_actualHeight.setDouble(getActualHeightMeters());
		nte_ffEffort.setDouble(m_ffEffort);
		nte_pdEffort.setDouble(m_pdEffort);
		nte_totalEffort.setDouble(m_totalEffort);
		nte_targetHeight.setDouble(getTargetHeightMeters());
		nte_atLowerLimit.setBoolean(isAtLowerLimit());
		nte_coast.setBoolean(m_isCoast);
	}
}
