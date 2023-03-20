package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.Conversions;
import frc.lib.swerve.CTREModuleState;
import frc.lib.util.DashboardManager;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.sim.TalonFXEncoderWrapper;
import frc.robot.Constants.SwerveK;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import static frc.robot.Constants.SwerveK.*;

import org.opencv.ml.KNearest;

public class SwerveModule {
    public final String moduleName;
    public final int moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private WPI_TalonFX m_steerMotor;
    private WPI_TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    private SwerveModuleState m_latestDesiredState = new SwerveModuleState();
    private double m_latestCmdedDriveVelo = 0;

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

    private final TalonFXEncoderWrapper m_driveMotorEncWrapper;
    private final TalonFXEncoderWrapper m_steerMotorEncWrapper;

    private double m_driveMotorSimDistance;
    private double m_steerMotorSimDistance;

    private final GenericEntry 
        nte_driveTemp,
        nte_steerTemp,
        nte_cancoderAngle,
        nte_modVelocity,
        nte_cancoderIntegratedAngle;
    private final GenericEntry
        nte_desiredStateVelocity,
        nte_desiredStateRotation,
        nte_actualStateVelocity,
        nte_actualStateRotation,
        nte_driveMotorVeloCmd;

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

        nte_desiredStateVelocity = DashboardManager.addTabItem(SwerveK.DB_TAB_NAME, moduleName + "/DesState/Velocity", 0);
        nte_desiredStateRotation = DashboardManager.addTabItem(SwerveK.DB_TAB_NAME, moduleName + "/DesState/Rotation", 0);
        nte_actualStateVelocity = DashboardManager.addTabItem(SwerveK.DB_TAB_NAME, moduleName + "/ActState/Velocity", 0);
        nte_actualStateRotation = DashboardManager.addTabItem(SwerveK.DB_TAB_NAME, moduleName + "/ActState/Rotation", 0);
        nte_driveMotorVeloCmd = DashboardManager.addTabItem(SwerveK.DB_TAB_NAME, moduleName + "/DriveMotorVeloCmd", 0);

        /* Angle Encoder Config */
        m_angleEncoder = new CANCoder(moduleConstants.cancoderID, "Canivore");
        configAngleEncoder();

        /* Angle Motor Config */
        m_steerMotor = new WPI_TalonFX(moduleConstants.angleMotorID, "Canivore");
        configAngleMotor();
        m_steerMotorEncWrapper = new TalonFXEncoderWrapper(m_steerMotor, 1.0 / kAngleGearRatio);

        /* Drive Motor Config */
        m_driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID, "Canivore");
        configDriveMotor();
        m_driveMotorEncWrapper = new TalonFXEncoderWrapper(m_driveMotor, (1.0 / kDriveGearRatio));

        m_lastAngle = getState().angle;
    }

    public void periodic() {
        nte_driveTemp.setDouble(m_driveMotor.getTemperature());
        nte_steerTemp.setDouble(m_steerMotor.getTemperature());
        nte_cancoderAngle.setDouble(m_angleEncoder.getAbsolutePosition());
        nte_modVelocity.setDouble(getState().speedMetersPerSecond);
        nte_cancoderIntegratedAngle.setDouble(getPosition().angle.getDegrees());

        // log states
        var curState = getState();
        nte_actualStateVelocity.setDouble(curState.speedMetersPerSecond);
        nte_actualStateRotation.setDouble(curState.angle.getDegrees());
        
        nte_desiredStateVelocity.setDouble(m_latestDesiredState.speedMetersPerSecond);
        nte_desiredStateRotation.setDouble(m_latestDesiredState.angle.getDegrees());

        nte_driveMotorVeloCmd.setDouble(m_latestCmdedDriveVelo);
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
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            m_latestCmdedDriveVelo = Conversions.MPSToFalcon(
                desiredState.speedMetersPerSecond, 
                kWheelCircumference_m,
                kDriveGearRatio
            );
            m_driveMotor.set(ControlMode.Velocity,
                m_latestCmdedDriveVelo,
                DemandType.ArbitraryFeedForward,
                kDriveFF.calculate(desiredState.speedMetersPerSecond)
            );
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean steerInPlace) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxVelocityMps * 0.01)) ? m_lastAngle
                : desiredState.angle;
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        if (!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            angle = m_lastAngle;
        }

        m_steerMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), kAngleGearRatio));
        m_lastAngle = angle;
    }

    private Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(m_steerMotorEncWrapper.getPosition());
    }

    public Rotation2d getCancoderAngle() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absPosDeg = 
            getCancoderAngle().getDegrees() < 0 ?
            makePositiveDegrees(getCancoderAngle().getDegrees()) - m_angleOffset.getDegrees() - 360 :
            makePositiveDegrees(getCancoderAngle().getDegrees()) - m_angleOffset.getDegrees();
        double absolutePosition = 360 / absPosDeg;
        m_steerMotorEncWrapper.setPosition(absolutePosition);
    }

    public void resetDriveToZero() {
        m_driveMotorEncWrapper.setPosition(0);
    }

    private void configAngleEncoder() {
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(CTREConfigs.Get().swerveCanCoderConfig);
    }

    public void brakeSteerMotor() {
        m_steerMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void coastDriveMotor() {
        m_driveMotor.setNeutralMode(NeutralMode.Coast);
    }

    private void configAngleMotor() {
        m_steerMotor.configFactoryDefault();
        m_steerMotor.configAllSettings(CTREConfigs.Get().swerveAngleFXConfig);
        m_steerMotor.setInverted(kInvertAngleMotor);
        m_steerMotor.setNeutralMode(kAngleNeutralMode);
        Timer.delay(0.1); // prevent init race condition
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(CTREConfigs.Get().swerveDriveFXConfig);
        m_driveMotor.setInverted(kInvertDriveMotor);
        m_driveMotor.setNeutralMode(kDriveNeutralMode);
        m_driveMotorEncWrapper.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveMotorEncWrapper.getVelocity() *  kWheelCircumference_m,
                getModuleAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveMotorEncWrapper.getPosition() * kWheelCircumference_m,
                getModuleAngle());
    }

    public void simulationPeriodic() {
        m_driveMotor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
        m_steerMotor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());

        double steerVolts = m_steerMotor.getSimCollection().getMotorOutputLeadVoltage();
        double driveVolts = m_driveMotor.getSimCollection().getMotorOutputLeadVoltage();
        m_steerMotorSim.setInputVoltage(steerVolts);
        m_driveMotorSim.setInputVoltage(driveVolts);

        m_steerMotorSim.update(0.02);
        m_driveMotorSim.update(0.02);

        double steerDegreesIncr = Units.radiansToDegrees(m_steerMotorSim.getAngularVelocityRadPerSec() * 0.02);
        double driveDegreesIncr = Units.radiansToDegrees(m_driveMotorSim.getAngularVelocityRadPerSec() * 0.02);

        m_steerMotorSimDistance += steerDegreesIncr;
        m_driveMotorSimDistance += driveDegreesIncr;

        double steerFalconTicks = Conversions.degreesToFalcon(m_steerMotorSimDistance, kAngleGearRatio);
        double driveFalconTicks = Conversions.degreesToFalcon(m_driveMotorSimDistance, kDriveGearRatio);

        m_steerMotor.getSimCollection().setIntegratedSensorRawPosition((int) steerFalconTicks);
        m_driveMotor.getSimCollection().setIntegratedSensorRawPosition((int) driveFalconTicks);

        double steerVelocity = m_steerMotorSim.getAngularVelocityRPM();
        double driveVelocity = m_driveMotorSim.getAngularVelocityRPM();
        double steerFalconVelocity = Conversions.RPMToFalcon(steerVelocity, kAngleGearRatio);
        double driveFalconVelocity = Conversions.RPMToFalcon(driveVelocity, kDriveGearRatio);

        m_steerMotor.getSimCollection().setIntegratedSensorVelocity((int) steerFalconVelocity);
        m_driveMotor.getSimCollection().setIntegratedSensorVelocity((int) driveFalconVelocity);
    }
}