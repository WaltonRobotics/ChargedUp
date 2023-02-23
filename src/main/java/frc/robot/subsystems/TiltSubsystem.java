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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.TiltK.*;
import static frc.robot.Constants.TiltK.kTiltMotorCANID;
import static frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

//TODO: Fix forwardlimit, fix coast,
public class TiltSubsystem extends SubsystemBase {
	private final CANSparkMax m_tiltMotor = new CANSparkMax(kTiltMotorCANID, MotorType.kBrushless);
	private final DutyCycleEncoder m_tiltAbsoluteEncoder = new DutyCycleEncoder(kTiltAbsoluteEncoderPort);
	private final Encoder m_tiltQuadratureEncoder = new Encoder(6, 7);
	private final DigitalInput m_homeSwitch = new DigitalInput(9);

	private final PIDController m_tiltController = new PIDController(kP, 0, kD);
	private double m_targetAngle = 0;
	private double m_ffEffort = 0; // TODO: calculate efforts
	private double m_pdEffort = 0;
	private double m_totalEffort = 0;

	private final GenericEntry nte_tiltMotorFFEffort, nte_tiltMotorPDEffort,
			nte_tiltMotorTotalEffort, nte_tiltTargetAngle,
			nte_tiltActualAngle, nte_coast, nte_homeSwitch, nte_forwardLimit;

	public TiltSubsystem() {
		m_tiltAbsoluteEncoder.reset();
		m_tiltMotor.setIdleMode(IdleMode.kBrake);
		m_tiltMotor.setSmartCurrentLimit(kTiltMotorCurrLimit);
		// m_tiltMotor.setSoftLimit(SoftLimitDirection.kForward, kAbsMaxDegree);

		// reset relative encoder on switch activation
		m_tiltQuadratureEncoder.setIndexSource(m_homeSwitch);

		DashboardManager.addTab(this);
		nte_tiltMotorPDEffort = DashboardManager.addTabDial(this, "TiltMotorPDEffort", -1, 1);
		nte_tiltMotorFFEffort = DashboardManager.addTabDial(this, "TiltMotorFFEffort", -1, 1);
		nte_tiltMotorTotalEffort = DashboardManager.addTabDial(this, "TiltMotorTotalEffort", -1, 1);
		nte_tiltTargetAngle = DashboardManager.addTabNumberBar(this, "TiltTargetAngle",
				kMinAngleDegrees, kMaxAngleDegrees);
		nte_tiltActualAngle = DashboardManager.addTabNumberBar(this, "TiltActualAngle", 0, 45);
		nte_coast = DashboardManager.addTabBooleanBox(this, "Tilt Coast");
		nte_homeSwitch = DashboardManager.addTabBooleanBox(this, "HomeSwitch");
		nte_forwardLimit = DashboardManager.addTabBooleanBox(this,"At Forward Limit");

		setCoast(false);
	}

	public CommandBase setTiltTarget(double degrees) {
		return runOnce(() -> i_setTiltTarget(degrees));
	}

	/*
	 * Return true if hitting max degree
	 */
	private boolean atForwardLimit() {
		if (m_tiltAbsoluteEncoder.get() >= kAbsMaxDegree) {
			return true;
		}
		return false;
	}

	/*
	 * Return true if at zero
	 */
	private boolean atReverseLimit() {
		return !m_homeSwitch.get();
	}

	private void i_setTiltTarget(double degrees) {
		m_targetAngle = MathUtil.clamp(degrees, 0, 45);
	}

	private double getDegrees() {
		var rawDeg = (m_tiltAbsoluteEncoder.get() * 360) - kAbsZeroDegreeOffset;
		return MathUtil.clamp(rawDeg, 0, 45); // get returns rotations, so rotations * (360 degrees / 1 rotation)
	}

	public CommandBase teleopTiltCmd(DoubleSupplier power) {
		return run(() -> {
			double powerVal = MathUtil.applyDeadband(power.getAsDouble(), stickDeadband);
			setTiltPower(powerVal);
		});
	}

	public void setTiltPower(double power) {
		double output;
		double dir = Math.signum(power);
		double powerVal = MathUtil.applyDeadband(power, stickDeadband);
		if ((atForwardLimit() && dir == 1) || (atReverseLimit() && dir == -1)) {
			output = 0;
		} else {
			output = powerVal;
		}
		m_tiltMotor.set(output);
	}

	public CommandBase toAngle(DoubleSupplier angle) {
		return run(() -> {
			m_targetAngle = angle.getAsDouble();
			// m_wristPDEffort = m_wristController.calculate(getDegrees(),
			// m_wristTargetAngle);
			m_totalEffort = m_ffEffort + m_pdEffort;

			setTiltPower(m_totalEffort);
		})
				.until(m_tiltController::atSetpoint)
				.withName("ToAngle");
	}

	private void setCoast(boolean coast) {
		if (coast) {
			m_tiltMotor.setIdleMode(IdleMode.kCoast);
		} else {
			m_tiltMotor.setIdleMode(IdleMode.kBrake);
		}
	}

	@Override
	public void periodic() {
		if(m_homeSwitch.get()) {

		}
		// // Set controller goal position
		// m_tiltController.setSetpoint(m_tiltTargetAngle);

		// // Calculate profile setpoint and effort
		m_pdEffort = m_tiltController.calculate(0);// TODO: conversion

		// // Calculate FF effort from profile setpoint
		double tiltFFEffort = 0; // TODO:Add FF
		// if (m_tiltController.getSetpoint() != 0) {
		tiltFFEffort = kFeedforward.calculate(m_tiltController.getSetpoint());
		// }
		// // Combine for total effort
		m_totalEffort = tiltFFEffort + m_pdEffort;
		// setCoast(nte_coast.get().getBoolean());
		updateShuffleBoard();
	}

	public void updateShuffleBoard() {
		// Push telemetry
		nte_tiltActualAngle.setDouble(getDegrees());// TODO: send actual converted anglee
		nte_tiltMotorFFEffort.setDouble(m_ffEffort);
		nte_tiltMotorPDEffort.setDouble(m_pdEffort);
		nte_tiltMotorTotalEffort.setDouble(m_totalEffort);
		nte_tiltTargetAngle.setDouble(m_targetAngle);
		nte_homeSwitch.setBoolean(atReverseLimit());
		nte_forwardLimit.setBoolean(atForwardLimit());

		SmartDashboard.putNumber("Tilt absolute position", m_tiltAbsoluteEncoder.get());
		SmartDashboard.putNumber("Tilt Quad Encoder position", m_tiltQuadratureEncoder.getRaw());
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
