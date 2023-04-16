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
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.Conversions;
import frc.robot.CTREConfigs;
import java.util.function.DoubleSupplier;

import frc.lib.logging.WaltLogger;
import frc.lib.logging.WaltLogger.*;

import static frc.robot.Constants.*;
public class ElevatorSubsystem extends SubsystemBase {
	private final WPI_TalonFX m_left = new WPI_TalonFX(kLeftCANID, canbus);
	private final WPI_TalonFX m_right = new WPI_TalonFX(kRightCANID, canbus);
	private final DigitalInput m_lowerLimit = new DigitalInput(kLowerLimitSwitchPort);
	private final Trigger m_lowerLimitTrigger = new Trigger(m_lowerLimit::get).negate();

	private final ProfiledPIDController m_controller = new ProfiledPIDController(
			kP, 0, kD, kConstraints);

	private final PIDController m_holdController = new PIDController(
		kPHold, 0, kDHold);

	private double m_targetHeight = 0;
	private double m_dynamicLowLimit = kMinHeightMeters;
	private double m_pdEffort = 0;
	private double m_ffEffort = 0;

	private double m_holdPdEffort = 0;
	private double m_holdFfEffort = 0;

	private boolean m_isCoast = false;

	private final DoubleLogger
		log_ffEffort, log_pdEffort, log_totalEffort, log_targetHeight, log_profileTargetHeight, 
		log_actualHeight, log_actualHeightRaw, log_profileVelo, log_actualVelo, 
		log_holdPdEffort, log_holdFfEffort;
	private final BooleanLogger log_atLowerLimit;

	private final GenericEntry nte_isCoast;

	public ElevatorSubsystem() {
		double subsysInitBegin = Timer.getFPGATimestamp();
		System.out.println("[INIT] ElevatorSubsystem Init Begin");
		m_left.configAllSettings(CTREConfigs.Get().leftConfig);
		m_right.configAllSettings(CTREConfigs.Get().rightConfig);

		m_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_right.configVoltageCompSaturation(kVoltageCompSaturationVolts);

		m_right.setNeutralMode(NeutralMode.Brake);
		m_left.setNeutralMode(NeutralMode.Brake);

		m_left.follow(m_right);
		m_left.setInverted(TalonFXInvertType.OpposeMaster);

		m_right.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
		m_right.configVelocityMeasurementWindow(16);

		log_ffEffort = WaltLogger.logDouble(DB_TAB_NAME, "FFEffort");
		log_pdEffort = WaltLogger.logDouble(DB_TAB_NAME, "PDEffort");
		log_totalEffort = WaltLogger.logDouble(DB_TAB_NAME, "TotalEffort");
		log_targetHeight = WaltLogger.logDouble(DB_TAB_NAME, "TargetHeightMeters");
		log_profileTargetHeight = WaltLogger.logDouble(DB_TAB_NAME, "ProfileTargetHeightMeters");
		log_actualHeight = WaltLogger.logDouble(DB_TAB_NAME, "ActualHeightMeters");
		log_actualHeightRaw = WaltLogger.logDouble(DB_TAB_NAME, "ActualHeightRaw");
		log_profileVelo = WaltLogger.logDouble(DB_TAB_NAME, "PDVelo");
		log_actualVelo = WaltLogger.logDouble(DB_TAB_NAME, "ActualVeloMPS");
		log_holdPdEffort = WaltLogger.logDouble(DB_TAB_NAME, "HoldPEffort");
		log_holdFfEffort = WaltLogger.logDouble(DB_TAB_NAME, "HoldFFEffort");
		log_atLowerLimit = WaltLogger.logBoolean(DB_TAB_NAME, "AtLowerLimit");

		m_lowerLimitTrigger.onTrue(Commands.runOnce(() -> {
			m_right.setSelectedSensorPosition(0);
		}).ignoringDisable(true));

		double subsysInitElapsed = Timer.getFPGATimestamp() - subsysInitBegin;
		System.out.println("[INIT] ElevatorSubsystem Init End: " + subsysInitElapsed + "s");

		nte_isCoast = Shuffleboard.getTab(DB_TAB_NAME)
                  .add("elev coast", false)
                  .withWidget(BuiltInWidgets.kToggleSwitch)
                  .getEntry();
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

	public CommandBase autoHome() {
		return Commands.sequence(
			startEnd(() -> {
				m_right.setVoltage(-2);
			}, () -> {
				m_right.setVoltage(0);
			}).until(m_lowerLimitTrigger)
		);
	}

	/**
	 * @return A cmd to move the elevator via stick
	 * sets elevator to target height if no input
	 */
	public CommandBase teleopCmd(DoubleSupplier power) {
		return run(() -> {
			double dir = Math.signum(power.getAsDouble());
			double output = 0;
			
			if (isFullyRetracted() && dir == -1) {
				output = 0;
			} else {
				output = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			}

			m_targetHeight += output * .02;
			double effort = getEffortForTarget(m_targetHeight);
			double holdEffort = getEffortToHold(m_targetHeight);
			
			if(output > 0){
				m_right.setVoltage(effort);
			} else {
				output = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
				m_right.setVoltage(holdEffort);
			}
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
	public CommandBase setCoast(boolean coast) {
		return runOnce(()-> { 
			m_left.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
			m_right.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
		});
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
	public double getEffortForTarget(double heightMeters) {
		m_pdEffort = m_controller.calculate(getActualHeightMeters(), heightMeters);
		m_ffEffort = 0;
		var pdSetpoint = m_controller.getSetpoint();
		if (pdSetpoint.velocity != 0) {
			m_ffEffort = kFeedforward.calculate(pdSetpoint.velocity);
		}
		double totalEffort = m_ffEffort + m_pdEffort;
		
		// logging
		log_ffEffort.accept(m_ffEffort);
		log_pdEffort.accept(m_pdEffort);
		log_totalEffort.accept(totalEffort);
		log_profileVelo.accept(pdSetpoint.velocity);
		log_profileTargetHeight.accept(pdSetpoint.position);

		return totalEffort;
	}

	private double getEffortToHold(double heightMeters) {
		m_holdPdEffort = m_holdController.calculate(getActualHeightMeters(), heightMeters);
		m_holdFfEffort = 0;
		var pdSetpoint = m_holdController.getSetpoint();
		if (pdSetpoint != 0) {
			m_holdFfEffort = kHoldKs;
		}
		double totalEffort = m_holdFfEffort + m_holdPdEffort;
		return totalEffort;
	}

	/**
	 * @return Cmd to move the elevator to specified height w/ pd & ff
	 * @param heightMeters The height to move to
	 */
	public CommandBase toHeight(double heightMeters) {
		return runOnce(() -> {
			m_controller.reset(getActualHeightMeters());
			i_setTarget(heightMeters);
			if(heightMeters < getActualHeightMeters()){
				m_controller.setConstraints(kConstraintsDown);
			}
			else{
				m_controller.setConstraints(kConstraints);
			}
		})
				.andThen(run(() -> {
					var effort = 
					MathUtil.clamp(getEffortForTarget(m_targetHeight), -kVoltageCompSaturationVolts,
							kVoltageCompSaturationVolts);
					
					m_right.set(ControlMode.PercentOutput, effort / kVoltageCompSaturationVolts);
				}))
				.until(() -> {
					return m_controller.atGoal();
				})
				.finallyDo((intr) -> {
					m_right.set(ControlMode.PercentOutput, 0);
				})
				.withName("AutoToHeight");
	}

	public CommandBase holdHeight(){
		return run(()->{
			var holdEffort = 
					MathUtil.clamp(getEffortToHold(m_targetHeight), -kVoltageCompSaturationVolts,
							kVoltageCompSaturationVolts);
			m_right.set(ControlMode.PercentOutput, holdEffort / kVoltageCompSaturationVolts);
		})
		.withName("Hold Height");
	}

	public enum ElevatorState {
		MAX(kMaxHeightMeters),
		SUBSTATION(kSubstationHeightM),
		TOPCONE(kTopConeHeightM),
		TOPCUBE(kTopCubeHeightM),
		MIDCONE(kMidConeHeightM),
		MIDCUBE(kMidCubeHeightM),
		MIN(kMinHeightMeters),
		EXTENDED_SUBSTATION(kExtendedSubstationHeightM);

		public double height;

		ElevatorState(double height) {
			this.height = height;
		}
	}

	@Override
	public void periodic() {
		updateShuffleBoard();
		log_holdPdEffort.accept(m_holdPdEffort);
		log_holdFfEffort.accept(m_holdFfEffort);
		setCoast(m_isCoast);
	}

	/*
	 * Updates nte values
	 */
	public void updateShuffleBoard() {
		log_actualHeightRaw.accept(getActualHeightRaw());
		log_actualHeight.accept(getActualHeightMeters());
		log_targetHeight.accept(getTargetHeightMeters());
		log_atLowerLimit.accept(isFullyRetracted());
		log_actualVelo.accept(getActualVelocityMps());
		SmartDashboard.putNumber("TICKS", getActualHeightRaw());
		SmartDashboard.putNumber("ACTUAL HEIGHT", getActualHeightMeters());
		m_isCoast = nte_isCoast.getBoolean(false);
	}
}
