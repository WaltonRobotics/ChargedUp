package com.pathplanner.lib;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;


import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;

public class PathPlannerUtil {
    private static final double FIELD_WIDTH_METERS = 8.02;
    private static final double FIELD_HEIGHT_METERS = 16.54;
    public static PathPlannerState transformStateForAlliance(
        PathPlannerState state, DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        // Create a new state so that we don't overwrite the original
        PathPlannerState transformedState = new PathPlannerState();
  
        Translation2d transformedTranslation =
            new Translation2d(FIELD_HEIGHT_METERS - state.poseMeters.getX(), FIELD_WIDTH_METERS - state.poseMeters.getY());
        Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
        Rotation2d transformedHolonomicRotation = state.holonomicRotation.times(-1);
  
        transformedState.timeSeconds = state.timeSeconds;
        transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
        transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
        transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
        transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
        transformedState.holonomicRotation = transformedHolonomicRotation;
        transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
        transformedState.curveRadius = -state.curveRadius;
        transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
        transformedState.deltaPos = state.deltaPos;
  
        return transformedState;
      } else {
        return state;
      }
    }
  
    public static PathPlannerTrajectory transformTrajectoryForAlliance(
        PathPlannerTrajectory trajectory, DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        List<State> transformedStates = new ArrayList<>();
  
        for (State s : trajectory.getStates()) {
          PathPlannerState state = (PathPlannerState) s;
  
          transformedStates.add(transformStateForAlliance(state, alliance));
        }

        Constructor<PathPlannerTrajectory> ctor = null;
        try {
          ctor = PathPlannerTrajectory.class.getDeclaredConstructor(
            List.class, List.class, StopEvent.class, StopEvent.class, Boolean.class
          );
          ctor.setAccessible(true);

          return ctor.newInstance(
            transformedStates,
            trajectory.getMarkers(),
            trajectory.getStartStopEvent(),
            trajectory.getEndStopEvent(),
            trajectory.fromGUI);
        } catch (Exception e) {
          e.printStackTrace();
        }

        return new PathPlannerTrajectory();
        
      } else {
        return trajectory;
      }
    }

    public static class PathPlannerState extends State {
    public double angularVelocityRadPerSec = 0;
    public Rotation2d holonomicRotation = new Rotation2d();
    public double holonomicAngularVelocityRadPerSec = 0;

    private double curveRadius = 0;
    private double deltaPos = 0;
    }
}
