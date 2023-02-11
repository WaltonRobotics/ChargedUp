package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.math.Conversions;
import frc.lib.util.DashboardManager;
import frc.robot.Constants.ElevatorLiftK;
import frc.robot.Constants.ElevatorTiltK;

public class Elevator extends SubsystemBase {
	
	// Motors
	private final WPI_TalonFX m_liftLeader = new WPI_TalonFX(ElevatorLiftK.kCANID); // change IDs later
	private final WPI_TalonFX m_liftFollower = new WPI_TalonFX(ElevatorLiftK.kCANID); // change IDs later
	private final WPI_TalonFX m_tiltMotor = new WPI_TalonFX(ElevatorTiltK.kCANID); // change IDs later

	// Sensors
	private final AnalogInput m_tiltPot = new AnalogInput(ElevatorTiltK.PotPort);
	
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

		m_liftLeader.getSensorCollection().setIntegratedSensorPosition(0, 0);

		nte_liftMotorFFEffort = DashboardManager.addTabDial(this, "LiftMotorFFEffort", -1, 1);
		nte_liftMotorPDEffort = DashboardManager.addTabDial(this, "LiftMotorPDEffort", -1, 1);
		nte_liftMotorTotalEffort = DashboardManager.addTabDial(this, "LiftMotorTotalEffort", -1, 1);
		nte_liftTargetHeight = DashboardManager.addTabNumberBar(this, "LiftTargetHeight",
			ElevatorLiftK.kMinHeightMeters, ElevatorLiftK.kMaxHeightMeters);
			nte_liftActualHeight = DashboardManager.addTabNumberBar(this, "LiftActualHeight",
			ElevatorLiftK.kMinHeightMeters, ElevatorLiftK.kMaxHeightMeters);

		DashboardManager.addTabSendable(this, "Lift Sim", m_mech2d);

		DashboardManager.addTab(this);

		m_tiltMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);

		nte_tiltMotorPDEffort = DashboardManager.addTabDial(this, "TiltMotorPDEffort", -1, 1);
		nte_tiltMotorTotalEffort = DashboardManager.addTabDial(this, "TiltMotorTotalEffort", -1, 1);
		nte_tiltTargetAngle = DashboardManager.addTabNumberBar(this, "TiltTargetAngle",
			ElevatorTiltK.kMinAngleDegrees, ElevatorTiltK.kMaxAngleDegrees);
		nte_tiltActualAngle = DashboardManager.addTabNumberBar(this, "TiltActualAngle", 0, 45);

		// TODO: instead of adding a Mechanism2d, tilt the lift Mechanism2d based on the tilt angle.
		// DashboardManager.addTabSendable(this, "Tilt", m_mech2d); 
	}

	private double falconToMeters() {
		var falconPos = m_liftLeader.getSelectedSensorPosition();
		var meters = Conversions.falconToMeters(
			falconPos, ElevatorLiftK.kDrumCircumferenceMeters , ElevatorLiftK.kGearRatio);
		return meters;
	}


	public CommandBase setLiftTarget(double meters) {
		return runOnce(() -> {
			i_setLiftTarget(meters);
		});
	}

	// public CommandBase setLiftTargetAdjustable(DoubleSupplier meters) {
	// 	return run(() -> {
	// 		i_setLiftTarget(meters.getAsDouble());
	// 	});
	// }

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
		// tilt
		// m_tiltController.setGoal(m_tiltTargetAngle);
		double tiltPDEffort = m_tiltController.calculate(potToDegrees(), m_tiltTargetAngle);
		// double tiltFFEffort = 0;
		// tiltFFEffort = ElevatorTiltK.kFeedforward.calculate();
		// }
		double tiltTotalEffort = tiltPDEffort;// + tiltFFEffort;
		m_tiltMotor.setVoltage(tiltTotalEffort);
		
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
		m_liftLeader.setVoltage(liftTotalEffort);
		m_liftFollower.follow(m_liftLeader);

		// Push telemetry
		nte_liftMotorFFEffort.setDouble(liftFFEffort);
		nte_liftMotorPDEffort.setDouble(liftPDEffort);
		nte_liftMotorTotalEffort.setDouble(liftTotalEffort);
		nte_liftActualHeight.setDouble(getLiftActualHeight());

		nte_tiltActualAngle.setDouble(getTiltActualDegrees());
		nte_tiltMotorPDEffort.setDouble(tiltPDEffort);
		nte_tiltMotorTotalEffort.setDouble(tiltTotalEffort);
		nte_tiltTargetAngle.setDouble(m_tiltTargetAngle);
	}

	@Override
	public void simulationPeriodic() {
		if (!DriverStation.isEnabled()) {
			return; // break out if not enabled. Robots can't move when disabled!
		}

		double motorVoltage = m_liftLeader.get() * m_liftLeader.getBusVoltage();
		m_liftSim.setInput(motorVoltage);
		m_liftSim.update(0.020);

		m_liftLeader.getSimCollection()
			.setIntegratedSensorRawPosition(
				(int)Conversions.MetersToFalcon(
					m_liftSim.getPositionMeters(),
					ElevatorLiftK.kDrumCircumferenceMeters,
					ElevatorLiftK.kGearRatio)
		);

		// TODO: move out so all sim objects can add their load together
		// RoboRioSim.setVInVoltage(
		// BatterySim.calculateDefaultBatteryLoadedVoltage(m_liftSim.getCurrentDrawAmps()));
		

		m_elevatorMech2d.setLength(Units.metersToInches(m_liftSim.getPositionMeters()));
	}

	// tilt
	// telemetry	
	private final GenericEntry /* nte_tiltMotorFFEffort, */ nte_tiltMotorPDEffort, 
                             nte_tiltMotorTotalEffort, nte_tiltTargetAngle,
							nte_tiltActualAngle;
	// control
	private final PIDController m_tiltController = new PIDController(
		ElevatorTiltK.kP, 0, ElevatorTiltK.kD);

	// target angle
	private double m_tiltTargetAngle = 0;

	public CommandBase setTiltTarget(double degrees) {
		return runOnce(() -> i_setTiltTarget(degrees));
	}

	private void i_setTiltTarget(double degrees) {
		// don't allow impossible angles :D

		m_tiltTargetAngle = MathUtil.clamp(degrees, 0, 45);
		
		// nte_liftTargetHeight.setDouble(m_liftTargetHeight);
	}

	private double potToDegrees()
	{
		return m_tiltPot.getVoltage(); // go back later
	}

	private double getTiltActualDegrees()
	{
		return potToDegrees();
	}
}