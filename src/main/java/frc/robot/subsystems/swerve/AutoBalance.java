package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
    private double rampDeg = 16; 
    private final SwerveSubsystem m_swerve;
    private boolean startedBalance = false;
    private boolean done = false;
    private double targetYaw;
    

    public AutoBalance(SwerveSubsystem swerve){
        m_swerve = swerve;
        addRequirements(swerve);
       
    }

    @Override
    public void initialize() {
        startedBalance = false;
        done = false;
        targetYaw = m_swerve.getGyroYaw();
        m_swerve.getThetaController().setTolerance(Math.toRadians(5));
        
        m_swerve.drive(3, 0, 0, false, true);
        // m_swerve.drive(0, 1.0, new Rotation2d(0,0), true);
    }

    @Override
    public void execute(){
        double pitch = m_swerve.getGyroPitch();
        
        if(pitch > rampDeg && !startedBalance){
            startedBalance = true;
        }

        if(startedBalance){
            if(pitch < 2.5){
                m_swerve.drive(0, 0, new Rotation2d(0,0), true);
                m_swerve.stopWithX();
                startedBalance = false;
                done = true;
            } else {
                double powerSign = pitch > 0 ? 1 : -1;
                double maxPitch = 30;
                double thetaEffort = m_swerve.getThetaController().calculate(Math.toRadians(m_swerve.getGyroYaw()), Math.toRadians(targetYaw));
                
                
                // Calculate a value between -1 and 1 based on the value of the min/max pitch
                // This does the inverse of a lerp and then copies the inverse of the sign of the pitch
                //double output_percentage = std::clamp(((units::math::abs(pitch) - min_pitch) / (max_pitch - min_pitch)).value(), -1.0, 1.0) * -wpi::sgn(pitch);
                double percentage = MathUtil.clamp(Math.abs(pitch) / maxPitch, 0.0, 0.65) * 4.75;
                // double percentage = std::clamp((units::math::abs(pitch) / maxPitch).value(), 0.0, 0.5);
                SmartDashboard.putNumber("AutoBal-Pct", percentage);
                // drivetrain.DriveRelative(percentage * powerSign);
                m_swerve.drive(percentage * powerSign, 0, thetaEffort, false, true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}