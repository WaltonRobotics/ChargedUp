package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorK.kConstraints;
import static frc.robot.Constants.ElevatorK.kElevatorD;
import static frc.robot.Constants.ElevatorK.kElevatorP;
import static frc.robot.Constants.ElevatorK.kLeftElevatorCANID;
import static frc.robot.Constants.ElevatorK.kRightElevatorCANID;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ElevatorK.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.DashboardManager;
import frc.robot.CTREConfigs;

public class ElevatorSubsystem extends SubsystemBase {
	private final WPI_TalonFX m_elevatorLeft = new WPI_TalonFX(kLeftElevatorCANID, canbus);
	private final WPI_TalonFX m_elevatorRight = new WPI_TalonFX(kRightElevatorCANID, canbus);
	private final DigitalInput m_lowerLimit = new DigitalInput(kLowerLimitSwitchPort);

	private final ProfiledPIDController m_elevatorController = new ProfiledPIDController(
			kElevatorP, 0, kElevatorD, kConstraints);

	private double m_targetHeight = 0;
	private double m_liftTotalEffort = 0;

	private final GenericEntry nte_liftMotorFFEffort, nte_liftMotorPDEffort,
			nte_liftMotorTotalEffort, nte_liftTargetHeight,
			nte_liftActualHeight, nte_atLowerLimit;

	private GenericEntry nte_coast = Shuffleboard.getTab("elevator lift idle mode")
			.add("coast", false)
			.withWidget("Toggle Button")
			.getEntry();

	public ElevatorSubsystem() {
		DashboardManager.addTab(this);

		m_elevatorLeft.configFactoryDefault();
		m_elevatorLeft.configAllSettings(CTREConfigs.Get().elevatorFXConfig);

		m_elevatorRight.configFactoryDefault();
		m_elevatorRight.configAllSettings(CTREConfigs.Get().elevatorFXConfig);

		//zero sensors
		m_elevatorRight.getSensorCollection().setIntegratedSensorPosition(0, 0);
		m_elevatorLeft.getSensorCollection().setIntegratedSensorPosition(0, 0);

		m_elevatorLeft.follow(m_elevatorRight);

		nte_liftMotorFFEffort = DashboardManager.addTabDial(this, "LiftMotorFFEffort", -1, 1);
		nte_liftMotorPDEffort = DashboardManager.addTabDial(this, "LiftMotorPDEffort", -1, 1);
		nte_liftMotorTotalEffort = DashboardManager.addTabDial(this, "LiftMotorTotalEffort", -1, 1);
		nte_liftTargetHeight = DashboardManager.addTabNumberBar(this, "LiftTargetHeight",
				kMinHeightMeters, kMaxHeightMeters);
		nte_liftActualHeight = DashboardManager.addTabNumberBar(this, "LiftActualHeight",
				kMinHeightMeters, kMaxHeightMeters);
		nte_atLowerLimit = DashboardManager.addTabBooleanBox(
			this, "At Lower Limit");
		DashboardManager.addTab(this);
	}

	public double getElevatorHeight() {
		return m_elevatorRight.getSelectedSensorPosition(0);
	}

	public boolean isAtLowerLimit() {
		return !m_lowerLimit.get();
	}

	private double falconToMeters() {
		var falconPos = m_elevatorRight.getSelectedSensorPosition();
		var meters = Conversions.falconToMeters(
				falconPos, kDrumCircumferenceMeters, kGearRatio);
		return meters;
	}

	public CommandBase teleOpElevatorCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			double dampener = .5;
			m_elevatorRight.set(ControlMode.PercentOutput, powerVal*dampener);
		});
	}

	private void i_setLiftTarget(double meters) {
		// don't allow impossible heights
		m_targetHeight = MathUtil.clamp(meters, kMinHeightMeters, kMaxHeightMeters);
	}

	//TODO: Convert to height
	private double getLiftActualHeight() {
		// return falconToMeters();
		return m_elevatorRight.getSelectedSensorPosition();
	}

	public double getTargetHeight(){
		return m_targetHeight;
	}

	private void setCoast(boolean coast) {
		if (coast) {
			m_elevatorLeft.setNeutralMode(NeutralMode.Coast);
			m_elevatorRight.setNeutralMode(NeutralMode.Coast);
		} else {
			m_elevatorLeft.setNeutralMode(NeutralMode.Brake);
			m_elevatorRight.setNeutralMode(NeutralMode.Brake);
		}
	}

	public CommandBase setLiftTarget(double meters) {
		return runOnce(() -> {
			i_setLiftTarget(meters);
		});
	}

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
		SmartDashboard.putNumber("Elevator Ticks", getElevatorHeight());
		nte_liftActualHeight.setDouble(getLiftActualHeight());
		//TODO: print out these values fr
		nte_liftMotorFFEffort.setDouble(0);
		nte_liftMotorPDEffort.setDouble(0);
		nte_liftMotorTotalEffort.setDouble(0);
		nte_liftTargetHeight.setDouble(getTargetHeight());
		nte_atLowerLimit.setBoolean(isAtLowerLimit());
		SmartDashboard.putBoolean("elevator lift idle mode", nte_coast.get().getBoolean());
	}
}
