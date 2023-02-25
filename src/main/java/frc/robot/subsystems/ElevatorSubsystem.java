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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

	//TODO: Actually set these values
	private double m_targetHeight = 0;
	private double m_pdEffort = 0;
	private double m_ffEffort = 0;
	private double m_totalEffort = 0;

	private final GenericEntry nte_ffEffort = DashboardManager.addTabDial(this, "LiftMotorFFEffort", -1, 1);
	private final GenericEntry nte_pdEffort = DashboardManager.addTabDial(this, "LiftMotorPDEffort", -1, 1);
	private final GenericEntry nte_totalEffort = DashboardManager.addTabDial(this, "LiftMotorTotalEffort", -1, 1);
	private final GenericEntry nte_targetHeight = DashboardManager.addTabNumberBar(this, "LiftTargetHeight",
			kMinHeightMeters, kMaxHeightMeters);
	private final GenericEntry nte_actualHeight = DashboardManager.addTabNumberBar(this, "LiftActualHeight",
		kMinHeightMeters, kMaxHeightMeters);
	private final GenericEntry nte_coast = DashboardManager.addTabBooleanToggle(this, "lift coast");
	private final GenericEntry nte_atLowerLimit = DashboardManager.addTabBooleanBox(this, "At Lower Limit");

	public ElevatorSubsystem() {
		// DashboardManager.addTab(this);
		
		m_left.configFactoryDefault();
        m_left.configAllSettings(CTREConfigs.Get().leftConfig);

		m_right.configFactoryDefault();
        m_right.configAllSettings(CTREConfigs.Get().rightConfig);

		m_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		// m_elevatorRight.setSelectedSensorPosition(0);

		m_right.setNeutralMode(NeutralMode.Brake);
		m_left.setNeutralMode(NeutralMode.Brake);

		m_left.follow(m_right);
		m_left.setInverted(TalonFXInvertType.OpposeMaster);
	}

	public double getHeight() {
		return m_right.getSelectedSensorPosition(0);
	}

	public boolean isAtLowerLimit() {
		return !m_lowerLimit.get();
	}

	private double falconToMeters() {
		var falconPos = m_right.getSelectedSensorPosition();
		var meters = Conversions.falconToMeters(
				falconPos, kDrumCircumferenceMeters, kGearRatio);
		return meters;
	}

	public CommandBase teleOpCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			m_right.set(ControlMode.PercentOutput, powerVal);
		});
	}



	private void i_setTarget(double meters) {
		// don't allow impossible heights
		m_targetHeight = MathUtil.clamp(meters, kMinHeightMeters, kMaxHeightMeters);
	}

	private double getActualHeight() {
		return falconToMeters() + 0.019;
		// return m_elevatorRight.getSelectedSensorPosition();
	}

	public double getTargetHeightRaw(){
		return m_targetHeight;
	}

	private void setCoast(boolean coast) {
		// m_elevatorLeft.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
		// m_elevatorRight.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
	}

	public CommandBase setTarget(double meters) {
		return runOnce(() -> {
			i_setTarget(meters);
		});
	}

	private double getTargetHeight() {
		return Conversions.MetersToFalcon(m_targetHeight, kDrumCircumferenceMeters, kGearRatio);
	}

	//TODO: Redo setmotot & gototarget methods
	// public CommandBase setMotors(double joystick) {
	// 	return run(() -> {
	// 		if (joystick == 0) {
	// 			while (getLiftActualHeight() > kMinHeightMeters) { 
	// 				m_elevatorRight.set(ControlMode.Velocity, -0.2);
	// 			}
	// 		}
	// 		m_elevatorRight.set(ControlMode.Velocity, kMaxVelocity * joystick);
	// 	});
	// }

	private double getEffortForTarget(double heightMeters) {
		double pdEffort = m_controller.calculate(falconToMeters(), heightMeters);

		double ffEffort = 0;

		if (m_controller.getSetpoint().velocity != 0) {
			ffEffort = kFeedforward.calculate(m_controller.getSetpoint().velocity);
		}

		double totalEffort = ffEffort + pdEffort;

		//setMotors(liftTotalEffort);

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

	public CommandBase setState(ElevatorStates state) {
		// switch (state) {
		// 	case MAX:
		// 		setLiftTarget(kMaxHeightMeters);
		// 		return runOnce(() -> getEffortForTarget());
		// 	case MID:
		// 		setLiftTarget(kMaxHeightMeters / 2);
		// 		return runOnce(() -> getEffortForTarget());
		// 	default:
		// 		setLiftTarget(kMinHeightMeters);
		// 		return runOnce(() -> getEffortForTarget());
		// }
		return Commands.none();
	}

	// public ElevatorState getState(double value) {
	// 	if (value <= 0.25) {
	// 		return ElevatorState.MIN;
	// 	} else if (value <= 0.75) {
	// 		return ElevatorState.MID;
	// 	}

	// 	return ElevatorState.MAX;
	// }

	public enum ElevatorStates {
		MAX,
		MID,
		MIN;
	}

	@Override
	public void periodic() {
		if(!m_lowerLimit.get()) {
			m_right.setSelectedSensorPosition(0);
		}
		updateShuffleBoard();
		setCoast(nte_coast.get().getBoolean());
	}

	public void updateShuffleBoard(){
		nte_actualHeight.setDouble(getActualHeight());
		nte_ffEffort.setDouble(m_ffEffort);
		nte_pdEffort.setDouble(m_pdEffort);
		nte_totalEffort.setDouble(m_totalEffort);
		nte_targetHeight.setDouble(getTargetHeightRaw());
		nte_atLowerLimit.setBoolean(isAtLowerLimit());
	}
}
