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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.TiltK.*;
import static frc.robot.Constants.TiltK.kTiltMotorCANID;


public class TiltSubsystem extends SubsystemBase {
	//lower limit: .555 ticks absolute
	//uppeer limit:  .635 ticks absolute
	private final CANSparkMax m_tiltMotor = new CANSparkMax(kTiltMotorCANID, MotorType.kBrushless);
	private final DutyCycleEncoder m_tiltAbsoluteEncoder = new DutyCycleEncoder(kTiltAbsoluteEncoderPort);
	private final Encoder m_tiltQuadratureEncoder = new Encoder(6, 7);
	private final DigitalInput m_tiltLimitSwitch = new DigitalInput(kTiltLimitSwitchPort);

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
		m_tiltAbsoluteEncoder.reset();
		m_tiltMotor.setSmartCurrentLimit(kTiltMotorCurrLimit);
		m_tiltMotor.setSoftLimit(SoftLimitDirection.kForward, kTiltMotorCurrLimit);

		//reset relative encoder on switch activation
		m_tiltQuadratureEncoder.setIndexSource(m_tiltLimitSwitch);
		DashboardManager.addTab(this);
		nte_tiltMotorPDEffort = DashboardManager.addTabDial(this, "TiltMotorPDEffort", -1, 1);
		nte_tiltMotorFFEffort = DashboardManager.addTabDial(this, "TiltMotorFFEffort", -1, 1);
		nte_tiltMotorTotalEffort = DashboardManager.addTabDial(this, "TiltMotorTotalEffort", -1, 1);
		nte_tiltTargetAngle = DashboardManager.addTabNumberBar(this, "TiltTargetAngle",
				kMinAngleDegrees, kMaxAngleDegrees);
		nte_tiltActualAngle = DashboardManager.addTabNumberBar(this, "TiltActualAngle", 0, 45);
	}

	//TODO:fix
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
		// // Set controller goal position
		// m_tiltController.setSetpoint(m_tiltTargetAngle);

		// // Calculate profile setpoint and effort
		double tiltPDEffort = m_tiltController.calculate(0);// TODO: conversion

		// // Calculate FF effort from profile setpoint
		double tiltFFEffort = 0;
		// if (m_tiltController.getSetpoint() != 0) {
			tiltFFEffort = kFeedforward.calculate(m_tiltController.getSetpoint());
		// }
		// // Combine for total effort
		double tiltTotalEffort = tiltFFEffort + tiltPDEffort;

		// // Command motor
		// m_tiltMotor.setVoltage(tiltTotalEffort);

		// Push telemetry
		nte_tiltActualAngle.setDouble(0);//fix
		nte_tiltMotorFFEffort.setDouble(tiltFFEffort);
		nte_tiltMotorPDEffort.setDouble(tiltPDEffort);
		nte_tiltMotorTotalEffort.setDouble(tiltTotalEffort);
		nte_tiltTargetAngle.setDouble(m_tiltTargetAngle);
		
		SmartDashboard.putBoolean("elevator tilt idle mode", nte_coast.get().getBoolean());

		SmartDashboard.putNumber("Tilt absolute position", m_tiltAbsoluteEncoder.get());
		SmartDashboard.putNumber("Tilt Quad Encoder position", m_tiltQuadratureEncoder.getRaw());
		setCoast(nte_coast.get().getBoolean());
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
