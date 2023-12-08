package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.SwerveK;

import static frc.robot.Constants.ElevatorK.*;

public final class CTREConfigs {
    private static final class Container {
        public static final CTREConfigs INSTANCE = new CTREConfigs();
    }

    public static CTREConfigs Get() {
        return Container.INSTANCE;
    }

    public final TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final CANcoderConfiguration swerveCancoderConfig = new CANcoderConfiguration();
    public final TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration leftConfig = new TalonFXConfiguration();

    private CTREConfigs() {
        /* Swerve Angle Motor Configurations */
        Slot0Configs angleSlot0Configs = new Slot0Configs();
        angleSlot0Configs.kP = Constants.SwerveK.kAngleKP;
        angleSlot0Configs.kI = Constants.SwerveK.kAngleKI;
        angleSlot0Configs.kD = Constants.SwerveK.kAngleKD;
        // TODO: figure out kV
        // angleSlot0Configs.kV = Constants.SwerveK.kAngleKF;
        swerveAngleFXConfig.Slot0 = angleSlot0Configs;
        swerveAngleFXConfig.MotorOutput.NeutralMode = SwerveK.kAngleNeutralMode;

        CurrentLimitsConfigs angleCurrentLimitsConfigs = new CurrentLimitsConfigs();
        angleCurrentLimitsConfigs.SupplyCurrentLimit = Constants.SwerveK.kAngleContinuousCurrentLimit;
        swerveAngleFXConfig.CurrentLimits = angleCurrentLimitsConfigs;

        /* Swerve Drive Motor Configuration */
        Slot0Configs driveSlot0Configs = new Slot0Configs();
        driveSlot0Configs.kP = Constants.SwerveK.kDriveKP;
        driveSlot0Configs.kI = Constants.SwerveK.kDriveKI;
        driveSlot0Configs.kD = Constants.SwerveK.kDriveKD;
        // TODO: figure out kV (again)
        // driveSlot0Configs.kV = Constants.SwerveK.kDriveKF;
        swerveDriveFXConfig.Slot0 = driveSlot0Configs;
        swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveK.kDriveNeutralMode;

        CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
        driveCurrentLimitsConfigs.SupplyCurrentLimit = Constants.SwerveK.kDriveContinuousCurrentLimit;
        swerveDriveFXConfig.CurrentLimits = driveCurrentLimitsConfigs;

        // TODO: check whether it is duty cycle or smtg else
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveK.kOpenLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SwerveK.kClosedLoopRamp;

        /* Swerve CANcoder Configuration */
        // TODO: check whether 0To1 works
        swerveCancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCancoderConfig.MagnetSensor.SensorDirection = Constants.SwerveK.kInvertCancoder
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

        // TODO: figure out what this is
        // swerveCancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Elevator Left and Right Motor Configuration */
        Slot0Configs rightSlot0Configs = new Slot0Configs();
        rightSlot0Configs.kP = kP;
        rightSlot0Configs.kD = kD;
        rightSlot0Configs.kV = kVoltageCompSaturationVolts / 100;
        rightConfig.Slot0 = rightSlot0Configs;

        rightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardLimit;
        rightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseLimit;
        rightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = kEnableForwardLimit;
        rightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = kEnableReverseLimit;

        CurrentLimitsConfigs elevCurrentLimitsConfigs = new CurrentLimitsConfigs();
        elevCurrentLimitsConfigs.SupplyCurrentLimit = kContinuousCurrentLimit;
        rightConfig.CurrentLimits = elevCurrentLimitsConfigs;
        leftConfig.CurrentLimits = elevCurrentLimitsConfigs;

        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
}