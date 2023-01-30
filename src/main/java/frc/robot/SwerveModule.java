package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.DashboardManager;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import static frc.robot.Constants.SwerveK.*;

public class SwerveModule {
    public final String moduleName;
    public final int moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    // Simulation
    private final FlywheelSim m_driveWheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            kDriveFF.kv * kWheelCircumference / (2*Math.PI),
            kDriveFF.ka * kWheelCircumference / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        kDriveGearRatio
    );
    private final FlywheelSim m_steeringSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kSteerFF.kv, kSteerFF.ka),
        DCMotor.getFalcon500(1),
        kAngleGearRatio
    );
    
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kDriveKS, kDriveKV, kDriveKA);

    private final GenericEntry nte_driveTemp, nte_steerTemp, nte_cancoderAngle, nte_modVelocity, nte_cancoderIntegratedAngle;

    public SwerveModule(String name, int moduleNumber, SwerveModuleConstants moduleConstants) {
        moduleName = name;
        this.moduleNumber = moduleNumber;
        this.m_angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        m_angleEncoder = new CANCoder(moduleConstants.cancoderID, "Canivore");
        configAngleEncoder();
        
        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID, "Canivore");
        configAngleMotor();
        
        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID, "Canivore");
        configDriveMotor();

        nte_driveTemp = DashboardManager.addTabDial("Swerve", moduleName + "/DriveTemp", 0, 100);
        nte_steerTemp = DashboardManager.addTabDial("Swerve", moduleName + "/SteerTemp", 0, 100);
        nte_cancoderAngle = DashboardManager.addTabItem("Swerve", moduleName + "/CancoderAngle", 0);
        nte_modVelocity = DashboardManager.addTabItem("Swerve", moduleName + "/ModuleVelocity", 0);
        nte_cancoderIntegratedAngle = DashboardManager.addTabItem("Swerve", moduleName + "/CancoderIntegratedAngle", 0);

        m_lastAngle = getState().angle;
    }

    public void periodic() {
        nte_driveTemp.setDouble(m_driveMotor.getTemperature());
        nte_steerTemp.setDouble(m_angleMotor.getTemperature());
        nte_cancoderAngle.setDouble(m_angleEncoder.getAbsolutePosition());
        nte_modVelocity.setDouble(getState().speedMetersPerSecond);
        nte_cancoderIntegratedAngle.setDouble(getPosition().angle.getDegrees());
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState, steerInPlace);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / kMaxVelocityMps;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, kWheelCircumference, kDriveGearRatio);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean steerInPlace) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxVelocityMps * 0.01)) ? m_lastAngle : desiredState.angle; 
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        if (!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            angle = m_lastAngle;
        }

        m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), kAngleGearRatio));
        m_lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), kAngleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_angleOffset.getDegrees(), kAngleGearRatio);
        m_angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(CTREConfigs.Get().swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        m_angleMotor.setInverted(Constants.SwerveK.kInvertAngleMotor);
        m_angleMotor.setNeutralMode(Constants.SwerveK.kAngleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(CTREConfigs.Get().swerveDriveFXConfig);
        m_driveMotor.setInverted(kInvertDriveMotor);
        m_driveMotor.setNeutralMode(kDriveNeutralMode);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), kWheelCircumference, kDriveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), kWheelCircumference, kDriveGearRatio), 
            getAngle()
        );
    }
}