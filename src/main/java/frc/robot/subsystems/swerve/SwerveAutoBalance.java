package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SwerveK.*;

  public class SwerveAutoBalance extends CommandBase {

    private final SwerveSubsystem m_swerve;
    private double angleDegrees;
  
    public SwerveAutoBalance(SwerveSubsystem swerve) {
      m_swerve = swerve;

      addRequirements(m_swerve);
    }
  
    @Override
    public void initialize() {
      angleDegrees = Double.POSITIVE_INFINITY;
    }
  
    @Override
    public void execute() {
      // Calculate charge station angle and velocity
      angleDegrees =
          m_swerve.getHeading().getCos() * m_swerve.getGyroPitch()
              + m_swerve.getHeading().getSin() * m_swerve.getGyroRoll();
      double angleVelocityDegreesPerSec =
          m_swerve.getHeading().getCos() * m_swerve.getPitchVelocity()
              + m_swerve.getHeading().getSin() * m_swerve.getRollVelocity();
      boolean shouldStop =
          (angleDegrees < 0.0 && angleVelocityDegreesPerSec > Units.radiansToDegrees(kMaxAngularVelocityRadps))
              || (angleDegrees > 0.0
                  && angleVelocityDegreesPerSec < -Units.radiansToDegrees(kMaxAngularVelocityRadps));
  
      // Send velocity to drive
      if (shouldStop) {
        m_swerve.drive(0,0, new Rotation2d(0,0), false);
      } else {
        m_swerve.drive(angleDegrees > 0 ? -0.5 : 0.5, 0, new Rotation2d(0), false);
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      m_swerve.stopWithX();
    }
  
    @Override
    public boolean isFinished() {
      return Math.abs(angleDegrees) < 2.5; // 2.5 = max degrees for the robot to be considered docked
    }
  }