package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.TiltK.*;
import static frc.robot.Constants.TiltK.kCANID;


//TODO: limit switch (hall-effect) 
public class TiltSubsystem extends SubsystemBase {
	private final CANSparkMax m_tiltMotor = new CANSparkMax(kCANID, MotorType.kBrushless);
	private final DutyCycleEncoder m_tiltAbsoluteEncoder = new DutyCycleEncoder(kTiltAbsoluteEncoderID);
	private final Encoder m_tiltQuadratureEncoder = new Encoder(6, 7);
	//add encoders
	private final PIDController m_tiltController = new PIDController(kP, 0, kD);
	private double m_tiltTargetAngle = 0;
	private final GenericEntry nte_tiltMotorFFEffort, nte_tiltMotorPDEffort,
			nte_tiltMotorTotalEffort, nte_tiltTargetAngle,
			nte_tiltActualAngle;
	private GenericEntry nte_coast = Shuffleboard.getTab("elevator tilt idle mode")
			.add("coast", false)
			.withWidget("Toggle Button")
			.getEntry();

	public TiltSubsystem() {
		m_tiltMotor.setSmartCurrentLimit(kTiltMotorCurrLimit);
		DashboardManager.addTab(this);
		nte_tiltMotorPDEffort = DashboardManager.addTabDial(this, "TiltMotorPDEffort", -1, 1);
		nte_tiltMotorFFEffort = DashboardManager.addTabDial(this, "TiltMotorFFEffort", -1, 1);
		nte_tiltMotorTotalEffort = DashboardManager.addTabDial(this, "TiltMotorTotalEffort", -1, 1);
		nte_tiltTargetAngle = DashboardManager.addTabNumberBar(this, "TiltTargetAngle",
				kMinAngleDegrees, kMaxAngleDegrees);
		nte_tiltActualAngle = DashboardManager.addTabNumberBar(this, "TiltActualAngle", 0, 45);
	}

	//fix
	double tiltPDEffort = m_tiltController.calculate(0, m_tiltTargetAngle);

	public CommandBase setTiltTarget(double degrees) {
		return runOnce(() -> i_setTiltTarget(degrees));
	}

	private void i_setTiltTarget(double degrees) {
		m_tiltTargetAngle = MathUtil.clamp(degrees, 0, 45);
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
		// Set controller goal position
		m_tiltController.setSetpoint(m_tiltTargetAngle);

		// Calculate profile setpoint and effort
		double tiltPDEffort = m_tiltController.calculate(0);// TODO: conversion

		// Calculate FF effort from profile setpoint
		double tiltFFEffort = 0;
		if (m_tiltController.getSetpoint() != 0) {
			tiltFFEffort = kFeedforward.calculate(m_tiltController.getSetpoint());
		}
		// Combine for total effort
		double tiltTotalEffort = tiltFFEffort + tiltPDEffort;

		// Command motor
		m_tiltMotor.setVoltage(tiltTotalEffort);

		// Push telemetry
		nte_tiltActualAngle.setDouble(0);//fix
		nte_tiltMotorFFEffort.setDouble(tiltFFEffort);
		nte_tiltMotorPDEffort.setDouble(tiltPDEffort);
		nte_tiltMotorTotalEffort.setDouble(tiltTotalEffort);
		nte_tiltTargetAngle.setDouble(m_tiltTargetAngle);

		setCoast(nte_coast.getBoolean(false));
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
