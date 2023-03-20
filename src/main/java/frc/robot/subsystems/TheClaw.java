package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.TheClawK.*;

public class TheClaw extends SubsystemBase {
	private final Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, kTheID);
	private final DigitalInput clawSensor = new DigitalInput(kClawSensor);
	private final GenericEntry nte_isClosed = DashboardManager.addTabBooleanBox(this, "Is Closed");
	private final GenericEntry nte_clawSensor = DashboardManager.addTabBooleanBox(this, "Claw Sensor");

	private final Timer m_lastCloseTimer = new Timer();
	
	private boolean m_isClosed = false;
	private boolean m_grabOk = false;
	
	public final Trigger sensorTrig = new Trigger(clawSensor::get).negate();
	public final Trigger grabOkTrig = new Trigger(() -> m_grabOk);

	public TheClaw() {
		DashboardManager.addTab(this);
		m_lastCloseTimer.reset();
		m_lastCloseTimer.start();
	}

	private void setClosed(boolean closed) {
		m_lastCloseTimer.restart();
		claw.set(!closed);
		m_isClosed = closed;
	}

	public CommandBase autoGrab(boolean autoGrab) {
		return run(()-> {
			m_grabOk = false;
			if (!m_isClosed && m_lastCloseTimer.hasElapsed(0.5)) {
				if (sensorTrig.getAsBoolean()){
					setClosed(true);
				}
			}
		})
		.until(() -> m_isClosed)
		.andThen(() -> {
			m_grabOk = true;
		})
		.repeatedly()
		.withName("DefaultAutoGrab");
	}

	/*
	 * @return Cmd to release claw
	 */
	public CommandBase release() {
		return runOnce(() -> {
			setClosed(false);
		} );
	}


	public CommandBase grab() {
		return runOnce(() -> {
			setClosed(true);
		});
	}

	public CommandBase getCmdForState(ClawState state){
		switch(state){
			case IGNORE: return Commands.none();
			case OPEN: return release();
			case CLOSE: return grab();
			case AUTO: return autoGrab(true);
		}
		return Commands.none();
	}

	public boolean getClosed() {
		return m_isClosed;
	}

	public enum ClawState{
		IGNORE, 
		OPEN,
		CLOSE,
		AUTO
	}
	@Override
	public void periodic() {
		nte_isClosed.setBoolean(m_isClosed);
		nte_clawSensor.setBoolean(sensorTrig.getAsBoolean());
	}
}
