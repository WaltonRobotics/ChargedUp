package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.TiltK.*;
import static frc.robot.Constants.TiltK.kMotorCANID;
import static frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

//TODO: Fix forwardlimit, fix coast,
public class TiltSubsystem extends SubsystemBase {
	private final CANSparkMax m_motor = new CANSparkMax(kMotorCANID, MotorType.kBrushless);
	private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(kAbsoluteEncoderPort);
	private final Encoder m_quadratureEncoder = new Encoder(kQuadEncoderA, kQuadEncoderB);
	private final DigitalInput m_homeSwitch = new DigitalInput(kHomeSwitchPort);

	private final PIDController m_controller = new PIDController(kP, 0, kD);
	private double m_targetAngle = 0;
	private double m_ffEffort = 0;
	private double m_pdEffort = 0;
	private double m_totalEffort = 0;
	private boolean m_isCoast = false;

	private final GenericEntry nte_motorPDEffort = DashboardManager.addTabDial(this, "MotorPDEffort", -1, 1);
	private final GenericEntry nte_motorFFEffort = DashboardManager.addTabDial(this, "TMotorFFEffort", -1, 1);
	private final GenericEntry nte_motorTotalEffort = DashboardManager.addTabDial(this, "MotorTotalEffort", -1, 1);
	private final GenericEntry nte_targetAngle = DashboardManager.addTabNumberBar(this, "TargetAngle",
			kMinAngleDegrees, kMaxAngleDegrees);
	private final GenericEntry nte_actualAngle = DashboardManager.addTabNumberBar(this, "ActualAngle", 0, 45);
	private final GenericEntry nte_coast = DashboardManager.addTabBooleanToggle(this, "coast");
	private final GenericEntry nte_homeSwitch = DashboardManager.addTabBooleanBox(this, "HomeSwitch");
	private final GenericEntry nte_forwardLimit = DashboardManager.addTabBooleanBox(this, "forward limit");

	public TiltSubsystem() {
		m_absoluteEncoder.reset();
		m_motor.setIdleMode(IdleMode.kBrake);
		m_motor.setSmartCurrentLimit(kMotorCurrLimit);

		// reset relative encoder on switch activation
		m_quadratureEncoder.setIndexSource(m_homeSwitch);
	}

	public CommandBase setTarget(double degrees) {
		return runOnce(() -> i_setTarget(degrees));
	}

	/*
	 * Return true if hitting max degree
	 */
	public boolean atForwardLimit() {
		if (getDegrees() >= kAbsMaxDegree) {
			return true;
		}
		return false;
	}

	/*
	 * Return true if at zero
	 */
	public boolean atReverseLimit() {
		return !m_homeSwitch.get();
	}

	private void i_setTarget(double degrees) {
		m_targetAngle = MathUtil.clamp(degrees, 0, 45);
	}

	private double getDegrees() {
		var rawDeg = (m_absoluteEncoder.get() * 360) - kAbsZeroDegreeOffset;
		return MathUtil.clamp(rawDeg, 0, 45); // get returns rotations, so rotations * (360 degrees / 1 rotation)
	}

	public CommandBase teleopCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			setPower(powerVal);
		});
	}

	public void setPower(double power) {
		double output;
		double dir = Math.signum(power);
		double powerVal = MathUtil.applyDeadband(power, stickDeadband);
		if ((atForwardLimit() && dir == 1) || (atReverseLimit() && dir == -1)) {
			output = 0;
		} else {
			output = powerVal;
		}
		m_motor.set(output);
	}

	public CommandBase toAngle(DoubleSupplier angle) {
		return run(() -> {
			m_targetAngle = angle.getAsDouble();
			m_pdEffort = m_controller.calculate(getDegrees(),
			m_targetAngle);
			m_totalEffort = m_ffEffort + m_pdEffort;

			setPower(m_totalEffort);
		})
				.until(m_controller::atSetpoint)
				.withName("ToAngle");
	}

	public CommandBase toAngle(double angle) {
		return run(() -> {
			m_targetAngle = angle;
			m_pdEffort = m_controller.calculate(getDegrees(),
			m_targetAngle);
			m_totalEffort = m_ffEffort + m_pdEffort;

			setPower(m_totalEffort);
		})
				.until(m_controller::atSetpoint)
				.withName("ToAngle");
	}

	private void setCoast(boolean coast) {
		if (coast) {
			m_isCoast = true;
			m_motor.setIdleMode(IdleMode.kCoast);
		} else {
			m_isCoast = false;
			m_motor.setIdleMode(IdleMode.kBrake);
		}
	}

	@Override
	public void periodic() {
		if(m_homeSwitch.get()) {

		}
		// // Set controller goal position
		// m_tiltController.setSetpoint(m_tiltTargetAngle);

		// // Calculate profile setpoint and effort
		m_pdEffort = m_controller.calculate(0);// TODO: conversion

		// // Calculate FF effort from profile setpoint
		double FFEffort = 0; // TODO:Add FF
		// if (m_tiltController.getSetpoint() != 0) {
		FFEffort = kFeedforward.calculate(m_controller.getSetpoint());
		// }
		// // Combine for total effort
		m_totalEffort = FFEffort + m_pdEffort;
		setCoast(nte_coast.get().getBoolean());
		updateShuffleBoard();
	}

	public void updateShuffleBoard() {
		// Push telemetry
		nte_actualAngle.setDouble(getDegrees());
		nte_motorFFEffort.setDouble(m_ffEffort);
		nte_motorPDEffort.setDouble(m_pdEffort);
		nte_motorTotalEffort.setDouble(m_totalEffort);
		nte_targetAngle.setDouble(m_targetAngle);
		nte_homeSwitch.setBoolean(atReverseLimit());
		nte_forwardLimit.setBoolean(atForwardLimit());
	}

	public CommandBase toState(TiltStates state) {
		return toAngle(state.angle); 
	}

	public enum TiltStates {
		MAX(kMaxAngleDegrees),
		HIGH(30),
		MID(kMaxAngleDegrees / 2),
		MIN(kMinAngleDegrees);

		public final double angle;

		private TiltStates(double angle) {
			this.angle = angle;
		}
	}
}
