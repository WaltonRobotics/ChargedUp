package frc.lib;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LoggedCommandXboxController extends CommandXboxController implements Loggable {

    private final String m_name;
    
    public LoggedCommandXboxController(int port, String name) {
        super(port);
        m_name = name;
    }

    @Override
    public String configureLogName() {
        return m_name + "XboxCtrl";
    }

    @Log @Override
    public double getLeftX() { return super.getLeftX(); }

    @Log @Override
    public double getLeftY() { return super.getLeftY(); }

    @Log @Override
    public double getRightX() { return super.getRightX(); }

    @Log @Override
    public double getRightY() { return super.getRightY(); }

    @Log @Override
    public double getLeftTriggerAxis() { return super.getLeftTriggerAxis(); }

    @Log @Override
    public double getRightTriggerAxis() { return super.getRightTriggerAxis(); }

    @Log @Override
    public Trigger a() { return super.a(); }
    
    @Log @Override
    public Trigger b() { return super.b(); }
    
    @Log @Override
    public Trigger x() { return super.x(); }

    @Log @Override
    public Trigger y() { return super.y(); }

    @Log @Override
    public Trigger start() { return super.start(); }
    
    @Log @Override
    public Trigger back() { return super.back(); }

    @Log @Override
    public Trigger leftTrigger() { return super.leftTrigger(); }

    @Log @Override
    public Trigger rightTrigger() { return super.rightTrigger(); }

    @Log @Override
    public Trigger leftBumper() { return super.leftBumper(); }

    @Log @Override
    public Trigger rightBumper() { return super.rightBumper(); }

    @Log @Override
    public Trigger leftStick() { return super.leftStick(); }

    @Log @Override
    public Trigger rightStick() { return super.rightStick(); }

    @Log @Override
    public Trigger povUp() { return super.povUp(); }

    @Log @Override
    public Trigger povDown() { return super.povDown(); }
    
    @Log @Override
    public Trigger povLeft() { return super.povLeft(); }

    @Log @Override
    public Trigger povRight() { return super.povRight(); }

    @Log @Override
    public Trigger povUpLeft() { return super.povUpLeft(); }

    @Log @Override
    public Trigger povUpRight() { return super.povUpRight(); }

    @Log @Override
    public Trigger povDownLeft() { return super.povDownLeft(); }

    @Log @Override
    public Trigger povDownRight() { return super.povDownRight(); }


}
