// package frc.robot.subsystems.swerve;

// import edu.wpi.first.math.filter.LinearFilter;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import static frc.robot.Constants.SwerveK.*;
// import frc.robot.Robot;

//   public class SwerveAutoBalance extends CommandBase {

//     private final SwerveSubsystem m_swerve;
//     private double angleDegrees;
  
//     public SwerveAutoBalance(SwerveSubsystem m_swerve) {
//       this.m_swerve = m_swerve;
//       addRequirements(m_swerve);
//     }
  
//     @Override
//     public void initialize() {
//       angleDegrees = Double.POSITIVE_INFINITY;
//     }
  
//     @Override
//     public void execute() {
//       // Calculate charge station angle and velocity
//       angleDegrees =
//           m_swerve.getHeading().getCos() * m_swerve.getGyroPitch()
//               + m_swerve.getHeading().getSin() * m_swerve.getGyroRoll();
//       double angleVelocityDegreesPerSec =
//           m_swerve.getHeading().getCos() * Units.radiansToDegrees(m_swerve.getPitchVelocity())
//               + m_swerve.getHeading().getSin() * Units.radiansToDegrees(m_swerve.getRollVelocity());
//       boolean shouldStop =
//           (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec.get())
//               || (angleDegrees > 0.0
//                   && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec.get());
  
//       // Send velocity to drive
//       if (shouldStop) {
//         m_swerve.drive(0,0, new Rotation2d(0,0), false);
//       } else {
//         m_swerve.runVelocity(
//             ChassisSpeeds.fromFieldRelativeSpeeds(
//                 Units.inchesToMeters(speedInchesPerSec.get()) * (angleDegrees > 0.0 ? -1.0 : 1.0),
//                 0.0,
//                 0.0,
//                 m_swerve.getHeading()));
//       }
//     }
  
//     @Override
//     public void end(boolean interrupted) {
//       m_swerve.stopWithX();
//     }
  
//     @Override
//     public boolean isFinished() {
//       return Math.abs(angleDegrees) < positionThresholdDegrees.get();
//     }
//   }