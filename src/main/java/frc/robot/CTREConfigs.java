package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveK.angleEnableCurrentLimit, 
            Constants.SwerveK.angleContinuousCurrentLimit, 
            Constants.SwerveK.anglePeakCurrentLimit, 
            Constants.SwerveK.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveK.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveK.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveK.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveK.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveK.driveEnableCurrentLimit, 
            Constants.SwerveK.driveContinuousCurrentLimit, 
            Constants.SwerveK.drivePeakCurrentLimit, 
            Constants.SwerveK.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveK.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveK.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveK.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveK.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveK.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveK.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveK.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}