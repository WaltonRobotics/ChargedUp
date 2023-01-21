// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autoncommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTrajectoryCommand extends CommandBase {
  private final SwerveSubsystem drivetrain = new SwerveSubsystem();
  private Trajectory trajectory = new Trajectory();

  public SwerveTrajectoryCommand(Trajectory trajectory) {
    addRequirements(drivetrain);
    this.trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetOdometry(trajectory.getInitialPose());
    new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose,
            Constants.SwerveK.kKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            drivetrain.getThetaController(),
            drivetrain::setModuleStates,
            drivetrain
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
