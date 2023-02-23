package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorK.kConstraints;
import static frc.robot.Constants.ElevatorK.kD;
import static frc.robot.Constants.ElevatorK.kP;
import static frc.robot.Constants.ElevatorK.kLeftCANID;
import static frc.robot.Constants.ElevatorK.kRightCANID;
import static frc.robot.Constants.ElevatorK.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
	private double m_totalEffort = 0;

	private final GenericEntry nte_motorFFEffort, nte_motorPDEffort, 
                             nte_motorTotalEffort, nte_targetHeight,
														 nte_actualHeight, 
														 nte_coast, nte_atLowerLimit;

	public ElevatorSubsystem() {
		DashboardManager.addTab(this);

		m_left.configFactoryDefault();
        m_left.configAllSettings(CTREConfigs.Get().elevatorFXConfig);

		m_right.configFactoryDefault();
        m_right.configAllSettings(CTREConfigs.Get().elevatorFXConfig);

		m_right.getSensorCollection().setIntegratedSensorPosition(0, 0);
		m_left.getSensorCollection().setIntegratedSensorPosition(0, 0);

		m_left.follow(m_right);

		nte_motorFFEffort = DashboardManager.addTabDial(this, "LiftMotorFFEffort", -1, 1);
		nte_motorPDEffort = DashboardManager.addTabDial(this, "LiftMotorPDEffort", -1, 1);
		nte_motorTotalEffort = DashboardManager.addTabDial(this, "LiftMotorTotalEffort", -1, 1);
		nte_targetHeight = DashboardManager.addTabNumberBar(this, "LiftTargetHeight",
				kMinHeightMeters, kMaxHeightMeters);
		nte_actualHeight = DashboardManager.addTabNumberBar(this, "LiftActualHeight",
			kMinHeightMeters, kMaxHeightMeters);
		nte_coast = DashboardManager.addTabBooleanBox(this, "lift coast");
		nte_atLowerLimit = DashboardManager.addTabBooleanBox(this, "At Lower Limit");
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
			double dampener = .5;
			m_right.set(ControlMode.PercentOutput, powerVal*dampener);
		});
	}

	private void i_setTarget(double meters) {
		// don't allow impossible heights
		m_targetHeight = MathUtil.clamp(meters, kMinHeightMeters, kMaxHeightMeters);
	}

	//TODO: Convert to height
	private double getActualHeight() {
		// return falconToMeters();
		return m_right.getSelectedSensorPosition();
	}

	public double getTargetHeight(){
		return m_targetHeight;
	}

	private void setCoast(boolean coast) {
		if (coast) {
			m_left.setNeutralMode(NeutralMode.Coast);
			m_right.setNeutralMode(NeutralMode.Coast);
		} else {
			m_left.setNeutralMode(NeutralMode.Brake);
			m_right.setNeutralMode(NeutralMode.Brake);
		}
	}

	public CommandBase setTarget(double meters) {
		return runOnce(() -> {
			i_setTarget(meters);
		});
	}

	private double getTarget() {
		return Conversions.MetersToFalcon(m_targetHeight, kDrumCircumferenceMeters, kGearRatio);
	}

	public CommandBase setMotors(double joystick) {
		return run(() -> {
			if (joystick == 0) {
				while (getActualHeight() > kMinHeightMeters) { 
					m_left.set(ControlMode.Velocity, -0.2);
					m_right.follow(m_left);
				}
			}
			m_left.set(ControlMode.Velocity, kMaxVelocity * joystick);
			m_right.follow(m_left);
		});
	}

	private void goToTarget() {
		double PDEffort = m_controller.calculate(falconToMeters());

		double FFEffort = 0;

		if (m_controller.getSetpoint().velocity != 0) {
			FFEffort = kFeedforward.calculate(m_controller.getSetpoint().velocity);
		}

		double totalEffort = FFEffort + PDEffort;

		setMotors(totalEffort);

		nte_motorFFEffort.setDouble(FFEffort);
		nte_motorPDEffort.setDouble(PDEffort);
		nte_motorTotalEffort.setDouble(totalEffort);
	}

	// private void zero() {
	// 	zeroing = true;
	// 	while (!atLowerPosition()) {
	// 		setMotors(-0.2);
	// 	}
	// 	zeroed = true;
	// 	zeroing = false;
	// }

	private boolean atLowerPosition() {
		double height = getActualHeight();
		return height <= kMinHeightMeters;
	}

	private boolean atUpperPosition() {
		double height = getActualHeight();
		return height >= kMaxHeightMeters;
	}

	// private void startZero() {
	// 	if (!zeroed) {
	// 		zero();
	// 	}
	// }

	public CommandBase setState(ElevatorStates state) {
		switch (state) {
			case MAX:
				setTarget(kMaxHeightMeters);
				return runOnce(() -> goToTarget());
			case MID:
				setTarget(kMaxHeightMeters / 2);
				return runOnce(() -> goToTarget());
			default:
				setTarget(kMinHeightMeters);
				return runOnce(() -> goToTarget());
		}
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
		updateShuffleBoard();
		setCoast(nte_coast.get().getBoolean());
	}

	public void updateShuffleBoard(){
		SmartDashboard.putNumber("Elevator Ticks", getHeight());
		nte_actualHeight.setDouble(getActualHeight());
		//TODO: print out these values fr
		nte_motorFFEffort.setDouble(0);
		nte_motorPDEffort.setDouble(0);
		nte_motorTotalEffort.setDouble(0);
		nte_targetHeight.setDouble(getTargetHeight());
		nte_atLowerLimit.setBoolean(isAtLowerLimit());
		SmartDashboard.putBoolean("elevator lift idle mode", nte_coast.get().getBoolean());
	}
}
