package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.TiltK.*;
import static frc.robot.Constants.TiltK.kMotorCANID;
import static frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

public class TiltSubsystem extends SubsystemBase {
	private final CANSparkMax m_motor = new CANSparkMax(kMotorCANID, MotorType.kBrushless);
	private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(kAbsoluteEncoderPort);
	private final Encoder m_quadratureEncoder = new Encoder(6, 7);
	private final DigitalInput m_limitSwitch = new DigitalInput(9);

	private final PIDController m_controller = new PIDController(kP, 0, kD);
	private double m_targetAngle = 0;
	private double m_FFEffort = 0;
	private double m_PDEffort = 0;
	private double m_totalEffort = 0;
	private final GenericEntry nte_tiltMotorFFEffort, nte_tiltMotorPDEffort,
			nte_tiltMotorTotalEffort, nte_tiltTargetAngle,
			nte_tiltActualAngle, nte_coast, nte_homeSwitch;

	public TiltSubsystem() {
		m_absoluteEncoder.reset();
		m_motor.setSmartCurrentLimit(kMotorCurrLimit);
		m_motor.setSoftLimit(SoftLimitDirection.kForward, kMotorCurrLimit);

		// reset relative encoder on switch activation
		m_quadratureEncoder.setIndexSource(m_limitSwitch);
		DashboardManager.addTab(this);
		nte_tiltMotorPDEffort = DashboardManager.addTabDial(this, "TiltMotorPDEffort", -1, 1);
		nte_tiltMotorFFEffort = DashboardManager.addTabDial(this, "TiltMotorFFEffort", -1, 1);
		nte_tiltMotorTotalEffort = DashboardManager.addTabDial(this, "TiltMotorTotalEffort", -1, 1);
		nte_tiltTargetAngle = DashboardManager.addTabNumberBar(this, "TiltTargetAngle",
				kMinAngleDegrees, kMaxAngleDegrees);
		nte_tiltActualAngle = DashboardManager.addTabNumberBar(this, "TiltActualAngle", 0, 45);
		nte_coast = DashboardManager.addTabBooleanBox(this, "tilt coast");
		nte_homeSwitch = DashboardManager.addTabBooleanBox(this, "HomeSwitch");
	}

	// TODO:fix
	double PDEffort = m_controller.calculate(0, m_targetAngle);

	public CommandBase setTarget(double degrees) {
		return runOnce(() -> i_setTarget(degrees));
	}

	private void i_setTarget(double degrees) {
		m_targetAngle = MathUtil.clamp(degrees, 0, 45);
	}

	private double getDegrees() {
		var rawDeg = (m_absoluteEncoder.get() * 360) - kAbsZeroDegreeOffset;
		return MathUtil.clamp(rawDeg, 0, 50); // get returns rotations, so rotations * (360 degrees / 1 rotation)
	}

	public CommandBase teleopCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			double dampener = .1;
			m_motor.set(powerVal * dampener);
		});
	}

	public boolean isAtZero(){
		return !m_limitSwitch.get();
	}

	private void setCoast(boolean coast) {
		if (coast) {
			m_motor.setIdleMode(IdleMode.kCoast);
		} else {
			m_motor.setIdleMode(IdleMode.kBrake);
		}
	}

	@Override
	public void periodic() {
		// // Set controller goal position
		// m_tiltController.setSetpoint(m_tiltTargetAngle);

		// // Calculate profile setpoint and effort
		m_PDEffort = m_controller.calculate(0);// TODO: conversion

		// // Calculate FF effort from profile setpoint
		double FFEffort = 0; // TODO:Add FF
		// if (m_tiltController.getSetpoint() != 0) {
		FFEffort = kFeedforward.calculate(m_controller.getSetpoint());
		// }
		// // Combine for total effort
		m_totalEffort = FFEffort + PDEffort;
		updateShuffleBoard();
		setCoast(nte_coast.get().getBoolean());
	}

	public void updateShuffleBoard() {
		// Push telemetry
		nte_tiltActualAngle.setDouble(getDegrees());//TODO: send actual converted anglee
		nte_tiltMotorFFEffort.setDouble(m_FFEffort);
		nte_tiltMotorPDEffort.setDouble(m_PDEffort);
		nte_tiltMotorTotalEffort.setDouble(m_totalEffort);
		nte_tiltTargetAngle.setDouble(m_targetAngle);
		nte_homeSwitch.setBoolean(isAtZero());

		SmartDashboard.putBoolean("elevator tilt idle mode", nte_coast.get().getBoolean());

		SmartDashboard.putNumber("Tilt absolute position", m_absoluteEncoder.get());
		SmartDashboard.putNumber("Tilt Quad Encoder position", m_quadratureEncoder.getRaw());
	}

	public enum States {
		MAX(kMaxAngleDegrees),
		HIGH(30),
		MID(kMaxAngleDegrees / 2),
		MIN(kMinAngleDegrees);

		public final double angle;

		private States(double angle) {
			this.angle = angle;
		}
	}
}
