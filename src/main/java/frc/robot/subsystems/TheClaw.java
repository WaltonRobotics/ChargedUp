package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DashboardManager;

import static frc.robot.Constants.TheClawK.*;

public class TheClaw extends SubsystemBase {
    private final Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, kID);
    private final DigitalInput leftEye = new DigitalInput(kLeftEyeID);
    private final DigitalInput rightEye = new DigitalInput(kRightEyeID);
    private boolean isClosed = false;
    private final GenericEntry nte_isClosed, nte_leftEye, nte_rightEye;

    public TheClaw(){
        DashboardManager.addTab(this);
        nte_isClosed = DashboardManager.addTabBooleanBox(this, "Is Claw Closed");
        nte_leftEye = DashboardManager.addTabBooleanBox(this, "Left Eye");
        nte_rightEye = DashboardManager.addTabBooleanBox(this, "Right Eye");
    }

    public void toggle(){
        claw.toggle();
        isClosed = !isClosed;
    }

    /**
     * snap the claw on sight 
     * release claw with no sight
     */
    public void handleSnap(){
        if((leftEye.get() || rightEye.get()) && !isClosed){
            toggle();
            isClosed = true;
        }
        if(!leftEye.get() && !rightEye.get() && isClosed){
            toggle();
        }
    }

    @Override
    public void periodic(){
       nte_isClosed.setBoolean(isClosed);
       nte_leftEye.setBoolean(leftEye.get());
       nte_rightEye.setBoolean(rightEye.get());
        // handleSnap();
    }
    
}
