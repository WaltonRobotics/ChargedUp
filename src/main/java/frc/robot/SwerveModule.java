package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.logging.WaltLogger;
import frc.lib.logging.WaltLogger.DoubleLogger;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveK;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.SwerveK.*;

public class SwerveModule {
    public final String moduleName;
    public final int moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_steerMotor;
    private TalonFX m_driveMotor;
    private CANcoder m_angleEncoder;

    private SwerveModuleState m_latestDesiredState = new SwerveModuleState();
    private double m_latestCmdedDriveVelo = 0;

    private final DoubleLogger log_driveTemp, log_steerTemp, log_cancoderAngle, log_modVelocity,
            log_steerInternalAngle, log_desiredStateVelocity, log_desiredStateRotation,
            log_actualStateVelocity, log_actualStateRotation, log_driveMotorVeloCmd, log_driveMotorPosition;

    public SwerveModule(String name, int moduleNumber, SwerveModuleConstants moduleConstants) {
        moduleName = name;
        this.moduleNumber = moduleNumber;
        this.m_angleOffset = moduleConstants.angleOffset;
        final String topicPrefix = SwerveK.DB_TAB_NAME + "/" + moduleName;

        log_driveTemp = WaltLogger.logDouble(topicPrefix, "DriveTemp");
        log_steerTemp = WaltLogger.logDouble(topicPrefix, "SteerTemp");
        log_cancoderAngle = WaltLogger.logDouble(topicPrefix, "CancoderAngle");
        log_modVelocity = WaltLogger.logDouble(topicPrefix, "ModuleVelocity");
        log_steerInternalAngle = WaltLogger.logDouble(topicPrefix, "SteerInternalAngle");
        log_desiredStateVelocity = WaltLogger.logDouble(topicPrefix, "DesState/Velocity");
        log_desiredStateRotation = WaltLogger.logDouble(topicPrefix, "DesState/Rotation");
        log_actualStateVelocity = WaltLogger.logDouble(topicPrefix, "ActState/Velocity");
        log_actualStateRotation = WaltLogger.logDouble(topicPrefix, "ActState/Rotation");
        log_driveMotorVeloCmd = WaltLogger.logDouble(topicPrefix, "DriveVeloCmd");
        log_driveMotorPosition = WaltLogger.logDouble(topicPrefix, "DriveTicks");

        /* Angle Encoder Config */
        m_angleEncoder = new CANcoder(moduleConstants.cancoderID, "Canivore");
        configAngleEncoder();

        /* Angle Motor Config */
        m_steerMotor = new TalonFX(moduleConstants.angleMotorID, "Canivore");
        configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID, "Canivore");
        configDriveMotor();

        m_lastAngle = getState().angle;
    }

    public void periodic() {
        log_driveTemp.accept(m_driveMotor.getDeviceTemp().getValue());
        log_steerTemp.accept(m_steerMotor.getDeviceTemp().getValue());
        log_cancoderAngle.accept(m_angleEncoder.getAbsolutePosition().getValue());
        log_modVelocity.accept(getState().speedMetersPerSecond);
        log_steerInternalAngle.accept(getPosition().angle.getDegrees());

        // log states
        var curState = getState();
        log_actualStateVelocity.accept(curState.speedMetersPerSecond);
        log_actualStateRotation.accept(curState.angle.getDegrees());

        log_desiredStateVelocity.accept(m_latestDesiredState.speedMetersPerSecond);
        log_desiredStateRotation.accept(m_latestDesiredState.angle.getDegrees());

        log_driveMotorVeloCmd.accept(m_latestCmdedDriveVelo);
        log_driveMotorPosition.accept(m_driveMotor.getRotorPosition().getValue());
    }

    public double makePositiveDegrees(double angle) {
        double degrees = angle;
        degrees = degrees % 360;
        if (degrees < 0.0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double makePositiveDegrees(Rotation2d angle) {
        return makePositiveDegrees(angle.getDegrees());
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle) {
        double steerAngle = makePositiveDegrees(newAngle);
        steerAngle %= (360);
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360) instead
        // of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;
        } else if (difference < -360) {
            steerAngle += 360;
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is > 90 deg < -90 deg the drive can be inverted so the
        // total
        // movement of the module is < 90 deg
        if (difference > 90 || difference < -90) {
            // Only need to add 180 deg here because the target angle will be put back into
            // the range [0, 2pi)
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * 
     * @param steerInPlace If modules should steer to target angle when target
     *                     velocity is 0
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState, steerInPlace);
        setSpeed(desiredState, isOpenLoop);

        m_latestDesiredState = desiredState;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kMaxVelocityMps;
            m_driveMotor.set(percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, kWheelCircumference,
                    kDriveGearRatio);
            m_latestCmdedDriveVelo = velocity;
            var request = new VelocityVoltage(0).withSlot(0);
            m_driveMotor.setControl(request.withVelocity(velocity)
                    .withFeedForward(kDriveFF.calculate(desiredState.speedMetersPerSecond)));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean steerInPlace) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxVelocityMps * 0.01)) ? m_lastAngle
                : desiredState.angle;
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        if (!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            angle = m_lastAngle;
        }

        m_steerMotor.setRPosition(Conversions.degreesToFalcon(angle.getDegrees(), kAngleGearRatio));
        m_lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d
                .fromDegrees(Conversions.falconToDegrees(m_steerMotor.getRotorPosition().getValue(), kAngleGearRatio));
    }

    public double getDriveMotorPosition() {
        return m_driveMotor.getRotorPosition().getValue();
    }

    public Rotation2d getCancoder() {
        return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        double cancoderDeg = getCancoder().getDegrees();
        double absPosDeg = cancoderDeg < 0 ? makePositiveDegrees(cancoderDeg) - m_angleOffset.getDegrees() - 360
                : makePositiveDegrees(cancoderDeg) - m_angleOffset.getDegrees();
        double absolutePosition = Conversions.degreesToFalcon(
                absPosDeg, kAngleGearRatio);
        m_steerMotor.setPosition(absolutePosition);
    }

    public void resetDriveToZero() {
        m_driveMotor.setPosition(0);
    }

    private void configAngleEncoder() {
        m_angleEncoder.getConfigurator().apply(CTREConfigs.Get().swerveCancoderConfig);
    }

    public void setOdoTestMode(boolean test) {
        CTREConfigs.Get().swerveAngleFXConfig.MotorOutput.NeutralMode = test ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        CTREConfigs.Get().swerveDriveFXConfig.MotorOutput.NeutralMode = test ? NeutralModeValue.Coast
                : NeutralModeValue.Brake;
    }

    private void configAngleMotor() {
        m_steerMotor.getConfigurator().apply(CTREConfigs.Get().swerveAngleFXConfig.Slot0);
        Timer.delay(0.1);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.getConfigurator().apply(CTREConfigs.Get().swerveDriveFXConfig.Slot0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(m_driveMotor.getRotorVelocity().getValue(), kWheelCircumference,
                        kDriveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(m_driveMotor.getRotorVelocity().getValue(), kWheelCircumference,
                        kDriveGearRatio),
                getAngle());
    }
}