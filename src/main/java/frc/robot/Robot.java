package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.FMSCacher;

public class Robot extends TimedRobot {

  private final Timer m_modResetTimer = new Timer();

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    if (!DriverStation.isFMSAttached()) {
      // Only run at home!
      PathPlannerServer.startServer(5811);
    }
    DriverStation.silenceJoystickConnectionWarning(true);
    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    
    
    m_robotContainer = new RobotContainer();
    addPeriodic(m_robotContainer.superstructure::periodicTelemetry, kDefaultPeriod);
  }

  @Override
  public void robotInit() {
    m_modResetTimer.restart();
    CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 320, 240, 120);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // if(FMSCacher.getCachedFMSAttached()) {
    //   NetworkTableInstance.getDefault().flush();
    // }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (m_modResetTimer.advanceIfElapsed(1)) {
      // m_robotContainer.swerve.resetToAbsolute();
    }
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.wrist.setCoast(false);
    m_robotContainer.elevator.setCoast(false);
    m_robotContainer.tilt.setCoast(false);
    m_robotContainer.superstructure.initState();
    m_robotContainer.tilt.resetEncoder();

    var initPoseOpt = m_robotContainer.getAutonomousInitPose();
    if (initPoseOpt.isPresent()) {
      m_robotContainer.swerve.resetPose(initPoseOpt.get());
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.wrist.setCoast(false);
    m_robotContainer.elevator.setCoast(false);
    m_robotContainer.tilt.setCoast(false);

    m_robotContainer.superstructure.initState();
    if(!DriverStation.isFMSAttached()){
      m_robotContainer.superstructure.smartReset();
    }
    m_robotContainer.tilt.autoHome();
    m_robotContainer.elevator.autoHome();
    m_robotContainer.swerve.resetToAbsolute();
    m_robotContainer.tilt.resetEncoder();

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
