package frc.team6995.lib.util.sim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Robot;

public class SparkMaxEncoderWrapper {
 
    private final RelativeEncoder sparkMaxEncoder;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;
  
    /**
     * Creates a new SparkMaxDerivedVelocityController using a default set of parameters.
     */
    public SparkMaxEncoderWrapper(CANSparkMax sparkMax) {
      this.sparkMaxEncoder = sparkMax.getEncoder();
    }

    /**
     * Returns the current position in rotations.
     */
    public double getPosition() {
        if(Robot.isReal()) {
            return sparkMaxEncoder.getPosition();
        } else {
            return simPosition;
        }
    }
  
    /**
     * Returns the current velocity in rotations per minute.
     */
    public synchronized double getVelocity() {
        if(Robot.isReal()) {
            return sparkMaxEncoder.getVelocity();
        } else {
            return simVelocity;
        }
    }

    public synchronized void setPosition(double position) {
        // we still want the encoder to report in motor shaft rotations, so divide by conversion factor.
        sparkMaxEncoder.setPosition(position);
        setSimPosition(position);
    }

    public synchronized void setSimPosition(double position) {
        simPosition = position;
    }

    public synchronized void setSimVelocity(double velocity) {
        simVelocity = velocity;
    }
  }
