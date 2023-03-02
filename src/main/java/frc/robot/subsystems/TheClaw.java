package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.DashboardManager;

import static frc.robot.Constants.TheClawK.*;

public class TheClaw extends SubsystemBase {
	private final Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, kTheID);
	private final DigitalInput leftEye = new DigitalInput(kLeftEyeID);
	private final DigitalInput rightEye = new DigitalInput(kRightEyeID);
	private boolean m_isClosed = false;
	private final GenericEntry nte_isClosed = DashboardManager.addTabBooleanBox(this, "Is Closed");
	private final GenericEntry nte_leftEye = DashboardManager.addTabBooleanBox(this, "Left Eye");
	private final GenericEntry nte_rightEye = DashboardManager.addTabBooleanBox(this, "Right Eye");

	public final Trigger leftEyeTrig = new Trigger(leftEye::get);
	public final Trigger rightEyeTrig = new Trigger(rightEye::get);

	public TheClaw() {
		// DashboardManager.addTab(this);

	}

	/*
	 * @return Cmd to automatically close claw on eye sight
	 */
	public CommandBase autoGrab(boolean autoRelease) {

		return runOnce(() ->  {
				m_isClosed = false;  // open claw
				claw.set(true); 
			})
			.andThen(new WaitCommand(leftEyeTrig.and(rightEyeTrig).getAsBoolean() ? 1.0 : 0.4)) // wait 0.8sec before sensor
			.andThen(
				startEnd(() -> {}, 
					() -> {
						m_isClosed = true;
						claw.set(false); 
					})
					.until(leftEyeTrig.and(rightEyeTrig)));
	}

	/*
	 * @return Cmd to release claw
	 */
	public CommandBase release() {
		return runOnce(() -> {
			m_isClosed = false;
			claw.set(true);
		} );
		
	}

	public CommandBase grab() {
		return runOnce(() -> {
			m_isClosed = true;
			claw.set(false);
		});
	}

	public CommandBase getCmdForState(ClawState state){
		switch(state){
			case IGNORE: return Commands.none();
			case OPEN: return release();
			case CLOSE: return grab();
			case AUTO: return autoGrab(m_isClosed);
		}
		return Commands.none();
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
		nte_leftEye.setBoolean(leftEye.get());
		nte_rightEye.setBoolean(rightEye.get());
	}
}
