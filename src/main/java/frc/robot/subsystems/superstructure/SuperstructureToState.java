package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperstructureToState extends SequentialCommandGroup {
    private final Superstructure m_superstructure;
    private final SuperState m_targetState;

    private BooleanSupplier m_wristWait = () -> true;
	private BooleanSupplier m_elevWait = () -> true;
	private BooleanSupplier m_tiltWait = () -> true;
    private BooleanSupplier m_clawWait = () -> true;

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
        });

        var prevState = m_superstructure.getPrevState();

        String quirks = "";

        if (m_targetState == SuperState.TOPCONE || m_targetState == SuperState.TOPCUBE) {
            m_elevWait = () -> (tilt.getDegrees() >= (m_targetState.tilt.angle*(.3)));
            m_wristWait = () -> (elevator.getActualHeightMeters() >= (m_targetState.elev.height*.3));
            quirks += "-TG_TOPCONECUBE";
        }

        if (m_targetState == SuperState.MIDCONE || m_targetState == SuperState.MIDCUBE) {
            m_elevWait = () -> (tilt.getDegrees() >= (m_targetState.tilt.angle*.25));
            m_wristWait = () -> (elevator.getActualHeightMeters() >= (m_targetState.elev.height*.25));
            quirks += "-TG_MIDCONECUBE";
        }

        if(m_targetState == SuperState.MIDCUBE || m_targetState == SuperState.TOPCUBE){
            m_clawWait = () -> (elevator.getActualHeightMeters() >= m_targetState.elev.height *.25);
            quirks += "-TG_CUBE";
        }

        if(m_targetState == SuperState.SAFE){
            m_elevWait = () -> (wrist.getDegrees() >= (m_targetState.wrist.angle - 25));
            m_tiltWait = ()-> (wrist.getDegrees() >= (m_targetState.wrist.angle - 20));
            quirks += "-TG_SAFE";
        }

        if(m_targetState == SuperState.SAFE && superstructure.m_prevState == SuperState.EXTENDED_SUBSTATION){
            m_elevWait = () -> (tilt.getDegrees() <= m_targetState.tilt.angle + 5);
            m_tiltWait = ()-> (wrist.getDegrees() >= (m_targetState.wrist.angle - 20));
            quirks += "-TG_SAFE__PV_EXTSUB";
        }

        if(m_targetState == SuperState.SAFE && (Math.abs(elevator.getActualHeightMeters() - SuperState.EXTENDED_SUBSTATION.elev.height) <= .2)
        && (Math.abs(tilt.getDegrees() - SuperState.EXTENDED_SUBSTATION.tilt.angle) <= 2)
        && (Math.abs(wrist.getDegrees() - SuperState.EXTENDED_SUBSTATION.wrist.angle) <= 2)){
            m_elevWait = () -> (tilt.getDegrees() <= m_targetState.tilt.angle + 5);
            m_tiltWait = ()-> (wrist.getDegrees() >= (m_targetState.wrist.angle - 20));
            quirks += "-TG_SAFE__PV_EXTSUB";
        }

        if(m_targetState == SuperState.SUBSTATION_PICK_UP){
            // m_elevWait = () -> (wrist.getDegrees() >= (m_targetState.wrist.angle * 0.1));
            m_wristWait = () -> (elevator.getActualHeightMeters() >= (m_targetState.elev.height * .25));
            quirks += "-TG_SUB";
        }

        CommandBase wristCmd = Commands.waitUntil(m_wristWait).andThen(wrist.toAngle(wristAngle));
		CommandBase elevCmd = Commands.waitUntil(m_elevWait).andThen(elevator.toHeight(m_targetState.elev.height));
		CommandBase tiltCmd = Commands.waitUntil(m_tiltWait).andThen(tilt.toAngle(m_targetState.tilt.angle));
		var clawCmd = Commands.waitUntil(m_clawWait).andThen(claw.getCmdForState(m_targetState.claw));
        var toSafe = m_superstructure.autoSafe();

		if (m_targetState == SuperState.GROUND_PICK_UP || m_targetState == SuperState.SUBSTATION_PICK_UP || m_targetState == SuperState.EXTENDED_SUBSTATION) {
			clawCmd = claw.release().andThen(claw.autoGrab(true));
		} 

        if(tilt.getDegrees() < 2 && (m_targetState == SuperState.SAFE || m_targetState == SuperState.GROUND_PICK_UP) || m_targetState == SuperState.SUBSTATION_PICK_UP){
            tiltCmd = Commands.none();
        }

        var fromStr = prevState.toString();
        var toStr = m_targetState.toString();
        final String fnQuirks = quirks;

        var dbgCmd = Commands.runOnce(() -> {
            superstructure.nte_stateQuirk.setString(fromStr + "-" + toStr + "-" + fnQuirks);
        });

        addCommands(
            initCmd,
            Commands.parallel(wristCmd, elevCmd, tiltCmd, clawCmd, dbgCmd)
        );


        setName("ToState-" + m_targetState.toString());
    }
}
