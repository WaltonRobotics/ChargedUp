package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.math.Conversions;
import frc.lib.util.DashboardManager;

import static frc.robot.Constants.ElevatorK.*;
import static frc.robot.Constants.ElevatorK.kLeftElevatorCANID;
import static frc.robot.Constants.ElevatorK.kRightElevatorCANID;

public class ElevatorSubsystem extends SubsystemBase {
	private final WPI_TalonFX m_elevatorLeft = new WPI_TalonFX(kLeftElevatorCANID);
	private final WPI_TalonFX m_elevatorRight = new WPI_TalonFX(kRightElevatorCANID);

	
	private final ProfiledPIDController m_elevatorController = new ProfiledPIDController(
		kP, 0, kD, kConstraints
	);

	private double m_liftTargetHeight;
	// private boolean zeroing;
	// private boolean zeroed;

	private final GenericEntry nte_liftMotorFFEffort, nte_liftMotorPDEffort, 
                             nte_liftMotorTotalEffort, nte_liftTargetHeight,
														 nte_liftActualHeight;

	public ElevatorSubsystem() {
		// zeroing = false;
		// zeroed = true;
		DashboardManager.addTab(this);

		m_elevatorRight.getSensorCollection().setIntegratedSensorPosition(0, 0);
		m_elevatorLeft.getSensorCollection().setIntegratedSensorPosition(0, 0);

		nte_liftMotorFFEffort = DashboardManager.addTabDial(this, "LiftMotorFFEffort", -1, 1);
		nte_liftMotorPDEffort = DashboardManager.addTabDial(this, "LiftMotorPDEffort", -1, 1);
		nte_liftMotorTotalEffort = DashboardManager.addTabDial(this, "LiftMotorTotalEffort", -1, 1);
		nte_liftTargetHeight = DashboardManager.addTabNumberBar(this, "LiftTargetHeight",
			kMinHeightMeters, kMaxHeightMeters);
		nte_liftActualHeight = DashboardManager.addTabNumberBar(this, "LiftActualHeight",
			kMinHeightMeters, kMaxHeightMeters);
		DashboardManager.addTab(this);
	}

	// @Override
	// public void periodic() {
	// 	setState();

	// 	nte_liftActualHeight.setDouble(getLiftActualHeight());
	// }

	private double falconToMeters() {
		var falconPos = m_elevatorRight.getSelectedSensorPosition();
		var meters = Conversions.falconToMeters(
			falconPos, kDrumCircumferenceMeters, kGearRatio);
		return meters;
	}

	public CommandBase setLiftTarget(double meters) {
		return runOnce(() -> {
			i_setLiftTarget(meters);
		});
	}

	private void i_setLiftTarget(double meters) {
		double height = getLiftActualHeight();
		// don't allow impossible heights
		m_liftTargetHeight = MathUtil.clamp(height, kMinHeightMeters, kMaxHeightMeters);
		nte_liftTargetHeight.setDouble(m_liftTargetHeight);
	}

	private double getLiftTarget() {
		return Conversions.MetersToFalcon(m_liftTargetHeight, kDrumCircumferenceMeters, kGearRatio);
	}

	private void setMotors(double voltage) {
		m_elevatorLeft.setVoltage(voltage);
		m_elevatorRight.follow(m_elevatorLeft);
	}

	private void goToTarget() {
		double liftPDEffort = m_elevatorController.calculate(falconToMeters());

		double liftFFEffort = 0;

		if (m_elevatorController.getSetpoint().velocity != 0) {
			liftFFEffort = kFeedforward.calculate(m_elevatorController.getSetpoint().velocity);
		}

		double liftTotalEffort = liftFFEffort + liftPDEffort;

		setMotors(liftTotalEffort);

		nte_liftMotorFFEffort.setDouble(liftFFEffort);
		nte_liftMotorPDEffort.setDouble(liftPDEffort);
		nte_liftMotorTotalEffort.setDouble(liftTotalEffort);
	}

	private double getLiftActualHeight() {
		return falconToMeters();
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
		double height = getLiftActualHeight();
		return height <= kMinHeightMeters;
	}

	private boolean atUpperPosition() {
		double height = getLiftActualHeight();
		return height >= kMaxHeightMeters;
	}

	// private void startZero() {
	// 	if (!zeroed) {
	// 		zero();
	// 	}
	// }

	private void setState(ElevatorStates state) {
		switch (state) {
			case MAX:
				setLiftTarget(ElevatorStates.MAX.height);
				goToTarget();
				break;
			case HIGH:
				setLiftTarget(ElevatorStates.HIGH.height);
				goToTarget();
				break;
			case MID:
				setLiftTarget(ElevatorStates.MID.height);
				goToTarget();
				break;
			case MIN: // default?
				setLiftTarget(ElevatorStates.MIN.height);
				goToTarget();
				break;
		}
	}

	public enum ElevatorStates {
		MAX(kMaxHeightMeters),
		HIGH(kMaxHeightMeters * 3 / 4),
		MID(kMaxHeightMeters / 2),
		MIN(kMinHeightMeters);

		public final double height;

		private ElevatorStates(double height) {
			this.height = height;
		}
	}
}