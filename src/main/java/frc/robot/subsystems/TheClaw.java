package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.logging.WaltLogger;
import frc.lib.logging.WaltLogger.DoubleLogger;
import frc.lib.util.DashboardManager;
import static frc.robot.Constants.TheClawK.*;

import java.sql.Driver;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


public class TheClaw extends SubsystemBase {
	private final Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, kTheID);
	private final DigitalInput clawSensor = new DigitalInput(kClawSensor);
	// private final GenericEntry nte_isClosed = DashboardManager.addTabBooleanBox(this, "Is Closed");
	// private final GenericEntry nte_clawSensor = DashboardManager.addTabBooleanBox(this, "Claw Sensor");
	// private final GenericEntry nte_superstate = DashboardManager.addTabItem(this, "ClawSuperState", "UNK");
	// private final GenericEntry nte_withinRange = DashboardManager.addTabBooleanBox(this, "within range");
	// private final GenericEntry nte_range = DashboardManager.addTabNumberBar(this, "time of flight range", 0, 1300);

	private final Supplier<ClawState> m_autoStateSupplier;
	private final Supplier<Double> m_wristDegSupplier;
	

	private final Timer m_lastActuationTimer = new Timer();
	private final Timer m_substationDelayTimer = new Timer();

	private static final double kSensorCheckDelay = .4;
	
	private boolean m_isClosed = false;
	private boolean m_grabOk = false;

	private final Servo m_leftExtend = new Servo(kLeftServo);
	private final Servo m_rightExtend = new Servo(kRightServo);
	
	public final Trigger sensorTrig = new Trigger(clawSensor::get).negate();
	public final Trigger closedTrig = new Trigger(() -> m_isClosed);
	public final Trigger grabOkTrig = new Trigger(() -> m_grabOk);
	private final Trigger stateAutoGrabTrig;
	private final Trigger substationStateAutoGrabTrig;
	private final Trigger extendedStateAutoGrabTrig;
	private final Trigger sensorCheckValidTrig = new Trigger(() -> m_lastActuationTimer.hasElapsed(kSensorCheckDelay));
	private final Trigger substationDelayTrig = new Trigger(() -> m_substationDelayTimer.hasElapsed(kSensorCheckDelay));
	private final Trigger wristAngleTrig;

	private final DoubleLogger log_leftExAngle = WaltLogger.logDouble(DB_TAB_NAME, "left extension angle");
	private final DoubleLogger log_rightExAngle = WaltLogger.logDouble(DB_TAB_NAME, "right extension angle");

	public TheClaw(Supplier<ClawState> autoStateSupplier, Supplier<Double> wristDegSupplier) {
		double subsysInitBegin = Timer.getFPGATimestamp();
		System.out.println("[INIT] ClawSubsystem Init Begin");
		m_autoStateSupplier = autoStateSupplier;
		// DashboardManager.addTab(this);
		m_lastActuationTimer.restart();
		m_wristDegSupplier = wristDegSupplier;
		m_substationDelayTimer.restart();
		wristAngleTrig = new Trigger(() -> m_wristDegSupplier.get().doubleValue() <= 50);


		stateAutoGrabTrig = new Trigger(() -> m_autoStateSupplier.get() == ClawState.AUTO);
		extendedStateAutoGrabTrig = new Trigger(()-> m_autoStateSupplier.get() == ClawState.EXTENDEDAUTO);
		substationStateAutoGrabTrig = new Trigger(() -> m_autoStateSupplier.get() == ClawState.SUBSTATIONAUTO);

    	var sensorAutonDebounceTrig = new Trigger(
			new BooleanSupplier() {
				final Debouncer m_debouncer = new Debouncer(0.025);

				@Override
				public boolean getAsBoolean() {
					if (DriverStation.isAutonomous()) {
						return m_debouncer.calculate(sensorTrig.getAsBoolean());
					} else {
						return sensorTrig.getAsBoolean();
					}
				}
       		}
		);

		stateAutoGrabTrig.onTrue(
			release()
			.andThen(runOnce(() -> m_grabOk = false)));
			
		stateAutoGrabTrig
		.and(sensorAutonDebounceTrig)
		.and(sensorCheckValidTrig)
			.onTrue(
				Commands.runOnce(() -> m_grabOk = true)
				.andThen(grab()).withName("internalAutoGrab")
		);

		extendedStateAutoGrabTrig.onTrue(
			release()
			.alongWith(extendExtra())
			.andThen(runOnce(() -> m_grabOk = false)));
			
		stateAutoGrabTrig
		.and(sensorAutonDebounceTrig)
		.and(sensorCheckValidTrig)
			.onTrue(
				Commands.runOnce(() -> m_grabOk = true)
				.andThen(grab().alongWith(retractExtra())).withName("internalExtendedAutoGrab")
		);

		

		substationStateAutoGrabTrig.onTrue(
			Commands.sequence(
				Commands.runOnce(()-> m_substationDelayTimer.restart()),
				Commands.waitUntil(wristAngleTrig),
				release(),
				Commands.runOnce(()-> m_substationDelayTimer.restart()),
				Commands.runOnce(() -> m_grabOk = false).withName("internalAutoGrabSSReset")
			)
		);
		substationStateAutoGrabTrig
		.and(sensorAutonDebounceTrig)
		// .and(timeOfFlightTrig)
		.and(substationDelayTrig)
		.and(closedTrig.negate())
		.onTrue(
			Commands.runOnce(() -> m_grabOk = true)
			.andThen(grab()).withName("internalAutoGrabSSAct")
		);
		

		// setDefaultCommand(autoGrab());
		double subsysInitElapsed = Timer.getFPGATimestamp() - subsysInitBegin;
		System.out.println("[INIT] ClawSubsystem Init End: " + subsysInitElapsed + "s");
	}

	private void setClosed(boolean closed) {
		m_lastActuationTimer.restart();
		m_substationDelayTimer.restart();
		claw.set(!closed);
		m_isClosed = closed;
	}

	// private boolean withinRange() {
	// 	return timeOfFlight.getRange() <= 177.8;
	// }

	public CommandBase autoGrab() {
		return Commands.none();
		// return run(()-> {
		// 	if(m_autoStateSupplier.get() == ClawState.AUTO){
		// 		setClosed(false);
		// 	}

		// 	m_grabOk = false;
		// 	if (!m_isClosed && m_lastCloseTimer.hasElapsed(0.5)) {
		// 		if (sensorTrig.getAsBoolean()){
		// 			setClosed(true);
		// 		}
		// 	}
		// })
		// .until(() -> m_isClosed)
		// .andThen(() -> {
		// 	m_grabOk = true;
		// })
		// .repeatedly()
		// .withName("DefaultAutoGrab");
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

	public boolean getClosed() {
		return m_isClosed;
	}

	public boolean getIsOkToGrab(){
		return m_grabOk;
	}

	public CommandBase retractExtra() {
		var rightRetract = new InstantCommand(() -> m_rightExtend.setPosition(kRetractIn));
		var leftRetract = new InstantCommand(() -> m_leftExtend.setPosition(kRetractIn));

		return Commands.parallel(
			rightRetract,
			leftRetract
		);
	}

	public CommandBase extendExtra() {
		var rightExtend = new InstantCommand(() -> m_rightExtend.setPosition(kExtendOut));
		var leftExtend = new InstantCommand(() -> m_leftExtend.setPosition(kExtendOut));

		return Commands.parallel(
			rightExtend,
			leftExtend
		);
	}

	public enum ClawState{
		IGNORE, 
		OPEN,
		CLOSE,
		AUTO,
		SUBSTATIONAUTO,
		EXTENDEDAUTO
	}

	@Override
	public void periodic() {
		// nte_isClosed.setBoolean(m_isClosed);
		// nte_clawSensor.setBoolean(sensorTrig.getAsBoolean());
		// nte_superstate.setString(m_autoStateSupplier.get().toString());
		// nte_withinRange.setBoolean(withinRange());
		// nte_range.setDouble(timeOfFlight.getRange());
		SmartDashboard.putBoolean("CLAW EYE", sensorTrig.getAsBoolean());
		log_leftExAngle.accept(m_leftExtend.getPosition());
		log_rightExAngle.accept(m_rightExtend.getPosition());
	}
}
