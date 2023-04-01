// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final Timer m_modResetTimer = new Timer();

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer = new RobotContainer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_modResetTimer.restart();
    if (!DriverStation.isFMSAttached()) {
      // Only run at home!
      PathPlannerServer.startServer(5811);
    }
    DriverStation.silenceJoystickConnectionWarning(true);
    addPeriodic(m_robotContainer.vision::periodic, .5);
    addPeriodic(m_robotContainer.superstructure::periodicTelemetry, kDefaultPeriod);

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    Logger.configureLoggingAndConfig(this, true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
    FieldConstants.updateAprilTags(m_robotContainer.swerve, m_robotContainer.vision.leftLowCam, m_robotContainer.vision.rightLowCam);
    NetworkTableInstance.getDefault().flush();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.swerve.stopWithX();
  }

  @Override
  public void disabledPeriodic() {
    if (m_modResetTimer.advanceIfElapsed(1)) {
      // m_robotContainer.swerve.resetToAbsolute();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.wrist.setCoast(false);
    m_robotContainer.elevator.setCoast(false);
    m_robotContainer.tilt.setCoast(false);
    m_robotContainer.superstructure.initState();

    var initPoseOpt = m_robotContainer.getAutonomousInitPose();
    if (initPoseOpt.isPresent()) {
      m_robotContainer.swerve.resetPose(initPoseOpt.get());
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.wrist.setCoast(false);
    m_robotContainer.elevator.setCoast(false);
    m_robotContainer.tilt.setCoast(false);

    m_robotContainer.superstructure.initState();
    //add if no fms, smartreset superstrucute
    if(!DriverStation.isFMSAttached()){
      m_robotContainer.superstructure.smartReset();
    }
    m_robotContainer.tilt.autoHome();
    m_robotContainer.elevator.autoHome();
    m_robotContainer.swerve.resetToAbsolute();
   
    m_robotContainer.swerve.setYaw(0); // Why?
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
