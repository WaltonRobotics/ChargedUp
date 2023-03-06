package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperstructureToState extends SequentialCommandGroup {
    private final Superstructure m_superstructure;
    private final SuperState m_targetState;

    private BooleanSupplier m_wristWait = () -> true;
	private BooleanSupplier m_elevWait = () -> true;
	private BooleanSupplier m_tiltWait = () -> true;

    private double wristAngle;

    public SuperstructureToState(Superstructure superstructure, SuperState targetState) {
        m_superstructure = superstructure;
        m_targetState = targetState;

        addRequirements(m_superstructure);
        var tilt = m_superstructure.m_tilt;
        var elevator = m_superstructure.m_elevator;
        var wrist = m_superstructure.m_wrist;
        var claw = m_superstructure.m_claw;
        
        wristAngle = m_targetState.wrist.angle;

        // set Superstructure internal state
        var initCmd = Commands.runOnce(() -> {
            m_superstructure.updateState(m_targetState);
            var prevState = m_superstructure.getPrevState();
        });

        var prevState = m_superstructure.getPrevState();
        var curState = m_superstructure.getCurState();

        if (m_targetState == SuperState.TOPCONE || m_targetState == SuperState.TOPCUBE) {
            m_elevWait = () -> (tilt.getDegrees() >= (m_targetState.tilt.angle*.25));
            m_wristWait = () -> (elevator.getActualHeightMeters() >= (m_targetState.elev.height*.25));
        }

        if (m_targetState == SuperState.MIDCONE || m_targetState == SuperState.MIDCUBE) {
            m_elevWait = () -> (tilt.getDegrees() >= (m_targetState.tilt.angle*.25));
            m_wristWait = () -> (elevator.getActualHeightMeters() >= (m_targetState.elev.height*.25));
        }

        if(m_targetState == SuperState.SAFE){
            m_elevWait = () -> (wrist.getDegrees() >= (m_targetState.wrist.angle *.90));
            m_tiltWait = ()-> (wrist.getDegrees() >= (m_targetState.wrist.angle - 30));
        }

        if(curState == SuperState.SUBSTATION_PICK_UP){
            m_elevWait = () -> (wrist.getDegrees() >= (m_targetState.wrist.angle * 0.1));
        }

        var wristCmd = Commands.waitUntil(m_wristWait).andThen(wrist.toAngle(wristAngle));
		var elevCmd = Commands.waitUntil(m_elevWait).andThen(elevator.toHeight(m_targetState.elev.height));
		var tiltCmd = Commands.waitUntil(m_tiltWait).andThen(tilt.toAngle(m_targetState.tilt.angle));
		var clawCmd = claw.getCmdForState(m_targetState.claw);

		if (m_targetState == SuperState.GROUND_PICK_UP || m_targetState == SuperState.SUBSTATION_PICK_UP) {
			clawCmd = claw.autoGrab(false);
		} 

        addCommands(
            initCmd,
            Commands.parallel(wristCmd, elevCmd, tiltCmd, clawCmd),
            Commands.run(()-> elevator.holdHeight(), elevator)
        );


        setName("ToState-" + m_targetState.toString());
    }
}
