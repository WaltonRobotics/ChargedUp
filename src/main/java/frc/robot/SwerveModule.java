package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.DashboardManager;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public final String moduleName;
    public final int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    private final GenericEntry nte_driveTemp, nte_steerTemp, nte_cancoderAngle, nte_modVelocity, nte_cancoderIntegratedAngle;


    public SwerveModule(String name, int moduleNumber, SwerveModuleConstants moduleConstants) {
        moduleName = name;
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, "Canivore");
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

        lastAngle = getState().angle;
    }

    public void periodic() {
        nte_driveTemp.setDouble(m_driveMotor.getTemperature());
        nte_steerTemp.setDouble(m_angleMotor.getTemperature());
        nte_cancoderAngle.setDouble(angleEncoder.getAbsolutePosition());
        nte_modVelocity.setDouble(getState().speedMetersPerSecond);
        nte_cancoderIntegratedAngle.setDouble(getPosition().angle.getDegrees());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        m_angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        m_angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        m_angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        m_driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        m_driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}