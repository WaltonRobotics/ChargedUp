package frc.robot;

import static frc.robot.Constants.ElevatorK.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

public final class CTREProConfigs {
    private static final class Container {
        public static final CTREProConfigs INSTANCE = new CTREProConfigs();
    }

    public static CTREProConfigs Get() {
        return Container.INSTANCE;
    }

    public final TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();
    

    private CTREProConfigs() {
        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveK.kAngleEnableCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = Constants.SwerveK.kAngleContinuousCurrentLimit;
        angleSupplyLimit.StatorCurrentLimitEnable = Constants.SwerveK.kAngleEnableCurrentLimit;
        angleSupplyLimit.StatorCurrentLimit = Constants.SwerveK.kAnglePeakCurrentLimit;

        
        swerveAngleFXConfig.Slot0.kP = Constants.SwerveK.kAngleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.SwerveK.kAngleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.SwerveK.kAngleKD;
        swerveAngleFXConfig.Slot0.kV = Constants.SwerveK.kAngleKV;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        swerveAngleFXConfig.MotorOutput.Inverted = 
            Constants.SwerveK.kInvertAngleMotor ? 
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        swerveAngleFXConfig.MotorOutput.NeutralMode = 
            Constants.SwerveK.kAngleNeutralMode == NeutralMode.Brake ? 
                NeutralModeValue.Brake : NeutralModeValue.Coast;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveK.kDriveEnableCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = Constants.SwerveK.kDriveContinuousCurrentLimit;
        driveSupplyLimit.StatorCurrentLimitEnable = Constants.SwerveK.kDriveEnableCurrentLimit;
        driveSupplyLimit.StatorCurrentLimit = Constants.SwerveK.kDrivePeakCurrentLimit;
     

        swerveDriveFXConfig.Slot0.kP = Constants.SwerveK.kDriveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.SwerveK.kDriveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.SwerveK.kDriveKD;
        swerveDriveFXConfig.Slot0.kV = Constants.SwerveK.kDriveKV;
        swerveDriveFXConfig.Slot0.kS = Constants.SwerveK.kDriveKS;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveK.kOpenLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveK.kOpenLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SwerveK.kClosedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.SwerveK.kClosedLoopRamp;

        swerveDriveFXConfig.MotorOutput.Inverted = 
            Constants.SwerveK.kInvertDriveMotor ? 
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;

        swerveDriveFXConfig.MotorOutput.NeutralMode = 
            Constants.SwerveK.kDriveNeutralMode == NeutralMode.Brake ? 
                NeutralModeValue.Brake : NeutralModeValue.Coast;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = (Constants.SwerveK.kInvertCanCoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive);
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

       
    }
}