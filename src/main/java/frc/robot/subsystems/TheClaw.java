package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;

import static frc.robot.Constants.TheClawK.*;

public class TheClaw extends SubsystemBase {
    private final Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, kTheClawID);
    private final DigitalInput leftCam = new DigitalInput(kLeftCamID);
    private final DigitalInput rightCam = new DigitalInput(kRightCamID);
    private boolean isClosed = false;
    private final GenericEntry nte_isClawClosed;

    public TheClaw(){
        DashboardManager.addTab(this);
        nte_isClawClosed = DashboardManager.addTabBooleanBox(this, "Is Claw Closed");
    }

    public boolean isClosed(){
        return isClosed();
    }

    public void toggleClaw(){
        claw.toggle();
        isClosed = !isClosed;
    }

    /**
     * snap the claw on sight 
     * open claw with no sight
     */
    public void handleSnap(){
        if((leftCam.get() || rightCam.get()) && !isClosed){
            toggleClaw();
            isClosed = true;
        }
        if(!leftCam.get() && !rightCam.get() && isClosed){
            toggleClaw();
        }
    }

    @Override
    public void periodic(){
       // nte_isClawClosed.setBoolean(isClosed());
        //handleSnap();
    }
    
}
