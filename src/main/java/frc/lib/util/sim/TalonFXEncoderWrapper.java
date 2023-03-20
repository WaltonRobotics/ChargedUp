package frc.lib.util.sim;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Robot;

public class TalonFXEncoderWrapper {
    private final TalonFX m_motor;
    private final double m_conversionFactor;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;

    // expects motor 
    public TalonFXEncoderWrapper(TalonFX motor, double conversionFactor) {
        m_motor = motor;
        m_conversionFactor = conversionFactor;
    }

    /**
     * @return Position in rotations
     */
    public double getPosition() {
        if (Robot.isReal()) {
            return (m_motor.getSelectedSensorVelocity() / 2048) * m_conversionFactor;
        } else {
            return simPosition * m_conversionFactor;
        }
    }

    /**
     * @return Velocity in rotations per second
     */
    public double getVelocity() {
        if (Robot.isReal()) {
            return m_motor.getSelectedSensorVelocity() / 2048 * 100;
        } else {
            return simVelocity;
        }
    }

    public synchronized void setSimPosition(double position) {
        simPosition = position;
    }

    public synchronized void setSimVelocity(double velocity) {
        simVelocity = velocity;
    }

    public synchronized void setPosition(double rotations) {
        m_motor.setSelectedSensorPosition(rotations * 2048);
        setSimPosition(rotations);
    }
}
