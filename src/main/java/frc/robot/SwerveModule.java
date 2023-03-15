package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.CTREProHelper;
import frc.lib.util.DashboardManager;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveK;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.StaticBrake;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import static frc.robot.Constants.SwerveK.*;

public class SwerveModule {
    public final String moduleName;
    public final int moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    // these are Phoenix Pro objects!!!!
    private TalonFX m_steerMotor;
    private TalonFX m_driveMotor;
    private CANcoder m_angleEncoder;

    private static final StaticBrake kBrakeRequest = new StaticBrake().withUpdateFreqHz(10);
    private static final DutyCycleOut kDutyCycleRequest = new DutyCycleOut(0, true, false);
    private static final VelocityVoltage kVelocityRequest = new VelocityVoltage(0, true, 0, 0, false);
    private static final PositionDutyCycle kPositionRequest = new PositionDutyCycle(0, true, 0, 0, false);

    // Physics

    // Simulation
    private final FlywheelSim m_driveMotorSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(kDriveFF.kv, kDriveFF.ka),
            DCMotor.getFalcon500(1),
            kDriveGearRatio);
    private final FlywheelSim m_steerMotorSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(kSteerFF.kv, kSteerFF.ka),
            DCMotor.getFalcon500(1),
            kAngleGearRatio);

    private double m_driveMotorSimDistance;
    private double m_steerMotorSimDistance;

    private final GenericEntry nte_driveTemp, nte_steerTemp, nte_cancoderAngle, nte_modVelocity,
            nte_cancoderIntegratedAngle;

    public SwerveModule(String name, int moduleNumber, SwerveModuleConstants moduleConstants) {
        moduleName = name;
        this.moduleNumber = moduleNumber;
        this.m_angleOffset = moduleConstants.angleOffset;

        nte_driveTemp = DashboardManager.addTabDial(SwerveK.DB_TAB_NAME, moduleName + "/DriveTemp", 0, 100);
        nte_steerTemp = DashboardManager.addTabDial(SwerveK.DB_TAB_NAME, moduleName + "/SteerTemp", 0, 100);
        nte_cancoderAngle = DashboardManager.addTabItem(SwerveK.DB_TAB_NAME, moduleName + "/CancoderAngle", 0);
        nte_modVelocity = DashboardManager.addTabItem(SwerveK.DB_TAB_NAME, moduleName + "/ModuleVelocity", 0);
        nte_cancoderIntegratedAngle = DashboardManager.addTabItem(SwerveK.DB_TAB_NAME,
                moduleName + "/CancoderIntegratedAngle", 0);

        /* Angle Encoder Config */
        m_angleEncoder = new CANcoder(moduleConstants.cancoderID, "Canivore");
        m_angleEncoder.getConfigurator().apply(CTREProConfigs.Get().swerveCanCoderConfig);

        /* Angle Motor Config */
        m_steerMotor = new TalonFX(moduleConstants.angleMotorID, "Canivore");
        m_steerMotor.getConfigurator().apply(CTREProConfigs.Get().swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID, "Canivore");
        m_driveMotor.getConfigurator().apply(CTREProConfigs.Get().swerveDriveFXConfig);
        resetDriveToZero();

        m_lastAngle = getState().angle;

        m_driveMotor.setControl(kBrakeRequest);
        m_steerMotor.setControl(kBrakeRequest);
    }

    public void periodic() {
        nte_driveTemp.setDouble(m_driveMotor.getDeviceTemp().getValue());
        nte_steerTemp.setDouble(m_steerMotor.getDeviceTemp().getValue());
        nte_cancoderAngle.setDouble(m_angleEncoder.getAbsolutePosition().getValue());
        nte_modVelocity.setDouble(getState().speedMetersPerSecond);
        nte_cancoderIntegratedAngle.setDouble(getPosition().angle.getDegrees());
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
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kMaxVelocityMps;

            m_driveMotor.setControl(kDutyCycleRequest.withOutput(percentOutput));
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, kWheelCircumference,
                    kDriveGearRatio);
            
            var req = kVelocityRequest
                .withVelocity(velocity)
                .withFeedForward(kDriveFF.calculate(desiredState.speedMetersPerSecond));
            m_driveMotor.setControl(req);
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean steerInPlace) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxVelocityMps * 0.01)) ? m_lastAngle
                : desiredState.angle;
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        if (!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            angle = m_lastAngle;
        }

        var req = kPositionRequest
            .withPosition(Conversions.degreesToFalconPro(angle.getDegrees(), kAngleGearRatio));
        m_steerMotor.setControl(req);
        m_lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d
                .fromDegrees(Conversions.falconToDegrees(m_steerMotor.getPosition().getValue(), kAngleGearRatio));
    }

    public double getDriveMotorPosition() {
        return m_driveMotor.getPosition().getValue();
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition().getValue() * 360);
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
                makePositiveDegrees(getCanCoder().getDegrees()) - m_angleOffset.getDegrees(), kAngleGearRatio);
        m_steerMotor.setRotorPosition(absolutePosition);
    }

    public void resetDriveToZero() {
        m_driveMotor.setRotorPosition(0);
    }

    public void brakeSteerMotor() {
        CTREProHelper.setNeutralMode(m_steerMotor, NeutralModeValue.Brake);
    }

    public void coastDriveMotor() {
        CTREProHelper.setNeutralMode(m_driveMotor, NeutralModeValue.Coast);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconProToMPS(m_driveMotor.getRotorVelocity().getValue(), kWheelCircumference, kDriveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconProToMeters(m_driveMotor.getRotorPosition().getValue(), kWheelCircumference,
                        kDriveGearRatio),
                getAngle());
    }

    public void simulationPeriodic() {
        // TODO: banks - convert for Pro
        // m_driveMotor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
        // m_steerMotor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());

        // double steerVolts = m_steerMotor.getSimCollection().getMotorOutputLeadVoltage();
        // double driveVolts = m_driveMotor.getSimCollection().getMotorOutputLeadVoltage();
        // m_steerMotorSim.setInputVoltage(steerVolts);
        // m_driveMotorSim.setInputVoltage(driveVolts);

        // m_steerMotorSim.update(0.02);
        // m_driveMotorSim.update(0.02);

        // double steerDegreesIncr = Units.radiansToDegrees(m_steerMotorSim.getAngularVelocityRadPerSec() * 0.02);
        // double driveDegreesIncr = Units.radiansToDegrees(m_driveMotorSim.getAngularVelocityRadPerSec() * 0.02);

        // m_steerMotorSimDistance += steerDegreesIncr;
        // m_driveMotorSimDistance += driveDegreesIncr;

        // double steerFalconTicks = Conversions.degreesToFalcon(m_steerMotorSimDistance, kAngleGearRatio);
        // double driveFalconTicks = Conversions.degreesToFalcon(m_driveMotorSimDistance, kDriveGearRatio);

        // m_steerMotor.getSimCollection().setIntegratedSensorRawPosition((int) steerFalconTicks);
        // m_driveMotor.getSimCollection().setIntegratedSensorRawPosition((int) driveFalconTicks);

        // double steerVelocity = m_steerMotorSim.getAngularVelocityRPM();
        // double driveVelocity = m_driveMotorSim.getAngularVelocityRPM();
        // double steerFalconVelocity = Conversions.RPMToFalcon(steerVelocity, kAngleGearRatio);
        // double driveFalconVelocity = Conversions.RPMToFalcon(driveVelocity, kDriveGearRatio);

        // m_steerMotor.getSimCollection().setIntegratedSensorVelocity((int) steerFalconVelocity);
        // m_driveMotor.getSimCollection().setIntegratedSensorVelocity((int) driveFalconVelocity);
    }
}