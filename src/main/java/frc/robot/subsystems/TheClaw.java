package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.DashboardManager;

import static frc.robot.Constants.TheClawK.*;

public class TheClaw extends SubsystemBase {
    private final Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, kTheClawID);
    private final DigitalInput leftEye = new DigitalInput(kLeftEyeID);
    private final DigitalInput rightEye = new DigitalInput(kRightEyeID);
    private boolean isClosed = false;
    private final GenericEntry nte_isClawClosed = DashboardManager.addTabBooleanBox(this, "Is Claw Closed");
    private final GenericEntry nte_leftEye = DashboardManager.addTabBooleanBox(this, "Left Eye");
    private final GenericEntry nte_rightEye = DashboardManager.addTabBooleanBox(this, "Right Eye");

    public final Trigger leftEyeTrig = new Trigger(leftEye::get);
    public final Trigger rightEyeTrig = new Trigger(rightEye::get);

    public TheClaw() {
        // DashboardManager.addTab(this);
    }

    public CommandBase autoGrab(boolean autoRelease) {

        return runOnce(() -> claw.set(true)) // open claw
            .withTimeout(leftEyeTrig.and(rightEyeTrig).getAsBoolean() ? 1.0 : 0.3) // wait 0.3sec before sensor
            .andThen(
                startEnd(()->{}, ()-> claw.set(false)).until(leftEyeTrig.and(rightEyeTrig))
            );
    }

    public CommandBase release() {
        return runOnce(() -> claw.set(true));
    }

    @Override
    public void periodic(){
       nte_isClawClosed.setBoolean(isClosed);
       nte_leftEye.setBoolean(leftEye.get());
       nte_rightEye.setBoolean(rightEye.get());
    }   
}
