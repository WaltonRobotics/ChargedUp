package com.pathplanner.lib;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PathPlannerUtil {
    public static final double FIELD_WIDTH_METERS = 8.02;
    public static final double FIELD_HEIGHT_METERS = 16.54;

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
        // transformedState.curveRadius = -state.curveRadius;
        transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
        // transformedState.deltaPos = state.deltaPos;
  
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
            List.class, List.class, StopEvent.class, StopEvent.class, boolean.class
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
    public static class Reflections {
      public static Translation2d reflectIfRed(Translation2d old) {
        if (DriverStation.getAlliance() == Alliance.Red) {
          return new Translation2d(FIELD_HEIGHT_METERS - old.getX(), old.getY());
        }
        return old;
      }
  
      public static double reflectIfRed(double x) {
        return reflectIfRed(new Translation2d(x, 0)).getX();
      }
  
      public static Rotation2d reflectIfRed(Rotation2d old) {
        if (DriverStation.getAlliance() == Alliance.Red) {
          return old.minus(Rotation2d.fromDegrees(180));
        }
        return old;
      }
    }
}
