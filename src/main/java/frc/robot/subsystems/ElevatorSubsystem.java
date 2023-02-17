package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.math.Conversions;
import frc.lib.util.DashboardManager;
import frc.robot.Constants.ElevatorK;

import static frc.robot.Constants.ElevatorK.*;
import static frc.robot.Constants.ElevatorK.kLeftElevatorCANID;
import static frc.robot.Constants.ElevatorK.kRightElevatorCANID;

public class ElevatorSubsystem extends SubsystemBase {
	
	// Motors
	// TODO: change ids in phoenix tuner to 10 and 11
	private final WPI_TalonFX m_elevatorLeft = new WPI_TalonFX(kLeftElevatorCANID); // change IDs later
	private final WPI_TalonFX m_elevatorRight = new WPI_TalonFX(kRightElevatorCANID); // change IDs later
	

	// Control
	private final ProfiledPIDController m_elevatorController = new ProfiledPIDController(
		kP, 0, kD, kConstraints
	);

	// State
	private double m_liftTargetHeight;

	private final GenericEntry nte_liftMotorFFEffort, nte_liftMotorPDEffort, 
                             nte_liftMotorTotalEffort, nte_liftTargetHeight,
														 nte_liftActualHeight;

	public ElevatorSubsystem() {
		DashboardManager.addTab(this);

		m_elevatorRight.getSensorCollection().setIntegratedSensorPosition(0, 0);
		m_elevatorLeft.getSensorCollection().setIntegratedSensorPosition(0, 0);

		nte_liftMotorFFEffort = DashboardManager.addTabDial(this, "LiftMotorFFEffort", -1, 1);
		nte_liftMotorPDEffort = DashboardManager.addTabDial(this, "LiftMotorPDEffort", -1, 1);
		nte_liftMotorTotalEffort = DashboardManager.addTabDial(this, "LiftMotorTotalEffort", -1, 1);
		nte_liftTargetHeight = DashboardManager.addTabNumberBar(this, "LiftTargetHeight",
			ElevatorK.kMinHeightMeters, ElevatorK.kMaxHeightMeters);
			nte_liftActualHeight = DashboardManager.addTabNumberBar(this, "LiftActualHeight",
			ElevatorK.kMinHeightMeters, ElevatorK.kMaxHeightMeters);
		DashboardManager.addTab(this);
	}

	private double falconToMeters() {
		var falconPos = m_elevatorRight.getSelectedSensorPosition();
		var meters = Conversions.falconToMeters(
			falconPos, ElevatorK.kDrumCircumferenceMeters , ElevatorK.kGearRatio);
		return meters;
	}


	public CommandBase setLiftTarget(double meters) {
		return runOnce(() -> {
			i_setLiftTarget(meters);
		});
	}

	private void i_setLiftTarget(double meters) {
		// don't allow impossible heights
		if (meters < ElevatorK.kMinHeightMeters) {
			meters = ElevatorK.kMinHeightMeters;
		}
		if (meters > ElevatorK.kMaxHeightMeters) {
			meters = ElevatorK.kMaxHeightMeters;
		}

		m_liftTargetHeight = meters;
		nte_liftTargetHeight.setDouble(m_liftTargetHeight);
	}

	private double getLiftTarget() {
		return Conversions.MetersToFalcon(m_liftTargetHeight, kDrumCircumferenceMeters, kGearRatio);
	}

	private void goToTarget() {
		double target = getLiftTarget();
		double height = getLiftActualHeight();
		double rate = m_elevatorController.calculate(height, target);
		setMotors(rate);
	}

	/**
	 * set motor speeds in m/s
	 * @param speed the speed
	 */
	private void setMotors(double speed) {
		m_elevatorLeft.set(ControlMode.Velocity, speed);
		m_elevatorRight.follow(m_elevatorLeft);
	}

	private double getLiftActualHeight() {
		return falconToMeters();
	}
}