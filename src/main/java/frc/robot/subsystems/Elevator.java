package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final Timer timer = new Timer();
	private boolean isMotionControlled;
	private double power;
	private boolean zeroing;
	private boolean zeroed;
	private double zeroStartTime;
    private boolean isReversed;

    private final TalonFX extensionController = new TalonFX(0); // change later
    private final TalonFX pivotController = new TalonFX(1); // change later
    
    public Elevator() {
		// zeroing = true;
    }

    public void setElevatorPower(double percent) {
        // percent = Math.min(0.75, Math.max(-0.75, percent)); // throttle power in
        power = percent;
        SmartDashboard.putNumber("Elevator Power", power);
        
        // System.out.println("Setting Power "  + percent);
        extensionController.set(ControlMode.PercentOutput, percent);
    }

    public void setReversed(boolean isReversed)
    {
        this.isReversed = isReversed;
    }

    public void setPivotPower(double percent) {
        // percent = Math.min(0.75, Math.max(-0.75, percent)); // throttle power in
        power = percent;
        SmartDashboard.putNumber("Pivot Power", power);
        
        // System.out.println("Setting Power "  + percent);
        pivotController.set(ControlMode.PercentOutput, percent);
    }

    public boolean isZeroed()
    {
        return zeroed;
    }

    public void setZeroed(boolean zeroed) 
    {
        this.zeroed = zeroed;
    }

    public double getPivotTemp()
    {
        return pivotController.getTemperature();
    }

    public double getExtensionTemp()
    {
        return extensionController.getTemperature();
    }

    public void alignVertical()
    {
        pivotController.set(ControlMode.Position, 0);   // check to see if this value is aligned vertically
    }

    public void periodic() {
        // if (zeroing) {
        //     setElevatorPower(-0.2);
        // }
    }
}