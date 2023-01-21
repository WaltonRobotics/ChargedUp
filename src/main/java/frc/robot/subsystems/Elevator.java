package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.DashboardManager;
import frc.lib.math.Conversions;
import frc.robot.Constants.ElevatorLiftK;
import frc.robot.Constants.ElevatorTiltK;

public class Elevator extends SubsystemBase {
	
	// Motors
	private final WPI_TalonFX m_liftMotor = new WPI_TalonFX(ElevatorLiftK.kCANID); // change IDs later
	private final WPI_TalonFX m_tiltMotor = new WPI_TalonFX(ElevatorTiltK.kCANID); // change IDs later
	
	// Control
	private final ProfiledPIDController m_liftController = new ProfiledPIDController(
		ElevatorLiftK.kP, 0, ElevatorLiftK.kD, ElevatorLiftK.kConstraints
	);

	// Simulation
	private final ElevatorSim m_liftSim = new ElevatorSim(
		ElevatorLiftK.kMotor, ElevatorLiftK.kGearRatio,
		ElevatorLiftK.kCarriageMassKg, ElevatorLiftK.kDrumRadiusMeters,
		ElevatorLiftK.kMinHeightMeters, ElevatorLiftK.kMaxHeightMeters,
true);

	// State
	private double m_liftTargetHeight = 0;

	private final GenericEntry nte_liftMotorFFEffort, nte_liftMotorPDEffort, 
                             nte_liftMotorTotalEffort, nte_liftTargetHeight,
														 nte_liftActualHeight;

	private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
	private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
	private final MechanismLigament2d m_elevatorMech2d =
	  m_mech2dRoot.append(
		  new MechanismLigament2d(
			  "Elevator", Units.metersToInches(m_liftSim.getPositionMeters()), 90));

	public Elevator() {
		DashboardManager.addTab(this);

		m_liftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

		nte_liftMotorFFEffort = DashboardManager.addTabDial(this, "LiftMotorFFEffort", -1, 1);
		nte_liftMotorPDEffort = DashboardManager.addTabDial(this, "LiftMotorPDEffort", -1, 1);
		nte_liftMotorTotalEffort = DashboardManager.addTabDial(this, "LiftMotorTotalEffort", -1, 1);
		nte_liftTargetHeight = DashboardManager.addTabNumberBar(this, "LiftTargetHeight",
			ElevatorLiftK.kMinHeightMeters, ElevatorLiftK.kMaxHeightMeters);
			nte_liftActualHeight = DashboardManager.addTabNumberBar(this, "LiftActualHeight",
			ElevatorLiftK.kMinHeightMeters, ElevatorLiftK.kMaxHeightMeters);

		DashboardManager.addTabSendable(this, "Lift Sim", m_mech2d);
	}

	private double falconToMeters() {
		var falconPos = m_liftMotor.getSelectedSensorPosition();
		var meters = Conversions.falconToMeters(
			falconPos, ElevatorLiftK.kDrumCircumferenceMeters , ElevatorLiftK.kGearRatio);
		return meters;
	}

	public CommandBase setLiftTarget(double meters) {
		return runOnce(()-> i_setLiftTarget(meters));
	}

	private void i_setLiftTarget(double meters) {
		// don't allow impossible heights
		if (meters < ElevatorLiftK.kMinHeightMeters) meters = ElevatorLiftK.kMinHeightMeters;
		if (meters > ElevatorLiftK.kMaxHeightMeters) meters = ElevatorLiftK.kMaxHeightMeters;

		m_liftTargetHeight = meters;
		nte_liftTargetHeight.setDouble(m_liftTargetHeight);
	}

	private double getLiftActualHeight() {
		return falconToMeters();
	}

	@Override
	public void periodic() {
		// Lift control

		// Set controller goal position
		m_liftController.setGoal(m_liftTargetHeight);

		// Calculate profile setpoint and effort
		double liftPDEffort = m_liftController.calculate(falconToMeters());

		// Calculate FF effort from profile setpoint
		double liftFFEffort = 0;
		if (m_liftController.getSetpoint().velocity != 0) {
			liftFFEffort = ElevatorLiftK.kFeedforward.calculate(m_liftController.getSetpoint().velocity);
		}
		// Combine for total effort
		double liftTotalEffort = liftFFEffort + liftPDEffort;

		// Command motor
		m_liftMotor.setVoltage(liftTotalEffort);

		// Push telemetry
		nte_liftMotorFFEffort.setDouble(liftFFEffort);
		nte_liftMotorPDEffort.setDouble(liftPDEffort);
		nte_liftMotorTotalEffort.setDouble(liftTotalEffort);
		nte_liftActualHeight.setDouble(getLiftActualHeight());
	}

	@Override
	public void simulationPeriodic() {
		if (!DriverStation.isEnabled()) {
			return; // break out if not enabled. Robots can't move when disabled!
		}

		double motorVoltage = m_liftMotor.get() * m_liftMotor.getBusVoltage();
		m_liftSim.setInput(motorVoltage);
		m_liftSim.update(0.020);

		m_liftMotor.getSimCollection()
			.setIntegratedSensorRawPosition(
				(int)Conversions.MetersToFalcon(
					m_liftSim.getPositionMeters(),
					ElevatorLiftK.kDrumCircumferenceMeters,
					ElevatorLiftK.kGearRatio)
		);


		// TODO: move out so all sim objects can add their load together
		// RoboRioSim.setVInVoltage(
		// 	BatterySim.calculateDefaultBatteryLoadedVoltage(m_liftSim.getCurrentDrawAmps())
		// );

		m_elevatorMech2d.setLength(Units.metersToInches(m_liftSim.getPositionMeters()));
	}
}