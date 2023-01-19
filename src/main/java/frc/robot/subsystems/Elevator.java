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

    private final TalonFX extensionController = new TalonFX(0); // change later

    public Elevator() {
		zeroing = true;
    }

    public void setPower(double percent) {
        // percent = Math.min(0.75, Math.max(-0.75, percent)); // throttle power in
        power = percent;
        SmartDashboard.putNumber("Elevator Power", power);
        
        // System.out.println("Setting Power "  + percent);
        extensionController.set(ControlMode.PercentOutput, percent);
    }

    public void periodic() {
        if (zeroing) {
            setPower(-0.2);
            // if (!elevatorLimitLower.get() || (Timer.getFPGATimestamp() - zeroStartTime > 1.0)) {
			// 	zeroing = false;
			// 	timer.stop();
			// 	enableControl();
			// 	zeroEncoder();
			// }
        }
    }
}