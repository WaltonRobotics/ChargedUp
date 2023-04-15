package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.Flipper;
import frc.robot.auton.AutonChooser;
import frc.robot.auton.AutonChooser.AutonOption;
import frc.robot.subsystems.superstructure.SuperState;

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
    double initBegin = Timer.getFPGATimestamp();
    System.out.println("[INIT] Robot Init Begin");
    m_modResetTimer.restart();
    if (DriverStation.isFMSAttached()) {
      Constants.kDebugLoggingEnabled = false;
    }
    if(m_robotContainer.tilt.getHomeSwitch()){
      m_robotContainer.tilt.resetEncoder().schedule();
    }
    double initElapsed = Timer.getFPGATimestamp() - initBegin;
		System.out.println("[INIT] Robot Init End: " + initElapsed + "s");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.wrist.setCoast(false);
    m_robotContainer.elevator.setCoast(false);
    m_robotContainer.tilt.setCoast(false);
    m_robotContainer.superstructure.initState();
   

    var initPoseOpt = m_robotContainer.getAutonomousInitPose();
    if (initPoseOpt.isPresent()) {
      m_robotContainer.swerve.resetPose(initPoseOpt.get());
      if(DriverStation.getAlliance() == Alliance.Blue){
          if (AutonChooser.GetChosenAutonCmd().equals(AutonChooser.GetAuton(AutonOption.TWO_ELEMENT_PARK)) || 
          AutonChooser.GetChosenAutonCmd().equals(AutonChooser.GetAuton(AutonOption.ONE_CONE_BUMP))) {
            m_robotContainer.swerve.setTeleOpGyroZero((initPoseOpt.get()).getRotation().getDegrees());
          } else {
            m_robotContainer.swerve.setTeleOpGyroZero((initPoseOpt.get()).getRotation().getDegrees() + 180);
          }
      }
      else{
          if (AutonChooser.GetChosenAutonCmd().equals(AutonChooser.GetAuton(AutonOption.TWO_ELEMENT_PARK)) || 
          AutonChooser.GetChosenAutonCmd().equals(AutonChooser.GetAuton(AutonOption.ONE_CONE_BUMP))) {
            m_robotContainer.swerve.setTeleOpGyroZero(Flipper.flipIfShould(initPoseOpt.get()).getRotation().getDegrees() + 180);
          } else {
            m_robotContainer.swerve.setTeleOpGyroZero(Flipper.flipIfShould(initPoseOpt.get()).getRotation().getDegrees());
          }
      }
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
    m_robotContainer.tilt.autoHome();
    m_robotContainer.elevator.autoHome();
    m_robotContainer.superstructure.toStateTeleop(SuperState.GROUND_PICK_UP);
    m_robotContainer.superstructure.toStateTeleop(SuperState.SAFE);

    m_robotContainer.superstructure.initState();

    if(!DriverStation.isFMSAttached()){
      m_robotContainer.superstructure.smartReset();
    }
    
   
    m_robotContainer.swerve.resetToAbsolute();
    m_robotContainer.swerve.setYaw(m_robotContainer.swerve.getTeleOpGyroZero());
    m_robotContainer.swerve.testModules();


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
