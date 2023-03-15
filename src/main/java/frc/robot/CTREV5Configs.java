package frc.robot;

import static frc.robot.Constants.ElevatorK.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public final class CTREV5Configs {
    private static final class Container {
        public static final CTREV5Configs INSTANCE = new CTREV5Configs();
    }

    public static CTREV5Configs Get() {
        return Container.INSTANCE;
    }
    
    /* Elevator Left and Right Motor Configuration */
    public final TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    

    private CTREV5Configs() {
        final SupplyCurrentLimitConfiguration elevatorSupplyLimit = new SupplyCurrentLimitConfiguration(
            kEnableCurrentLimit,
            kContinuousCurrentLimit,
            kPeakCurrentLimit,
            kPeakCurrentDuration
        );

        rightConfig.slot0.kP = kP;
        rightConfig.slot0.kD = kD;
        rightConfig.reverseSoftLimitThreshold = kReverseLimit;
        rightConfig.forwardSoftLimitThreshold = kForwardLimit;
        rightConfig.forwardSoftLimitEnable = kEnableForwardLimit;
        //rightConfig.reverseSoftLimitEnable = kEnableReverseLimit;

        rightConfig.supplyCurrLimit = elevatorSupplyLimit;
        leftConfig.supplyCurrLimit = elevatorSupplyLimit;
    }
}