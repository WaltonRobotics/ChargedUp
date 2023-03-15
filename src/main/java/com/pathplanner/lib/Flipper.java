package com.pathplanner.lib;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

import static frc.robot.FieldConstants.*;


public class Flipper {

    private static Constructor<PathPlannerTrajectory> constructor;
    
    static {
        try {
            // Set all `.deltaPos` on PathPlannerState objects to be public
            // deltaPosField = PathPlannerState.class.getDeclaredField("deltaPos");
            // deltaPosField.setAccessible(true);

            // Set all `.curveRadius` on PathPlannerState objects to be public
            // curveRadiusField = PathPlannerState.class.getDeclaredField("curveRadius");
            // curveRadiusField.setAccessible(true);

            // Access the private constructor that builds a trajectory from states
            constructor = PathPlannerTrajectory.class.getDeclaredConstructor(
                    List.class, List.class, StopEvent.class, StopEvent.class, boolean.class);
            constructor.setAccessible(true);
        } catch (SecurityException | NoSuchMethodException e) {
            System.err.println(
                    "Could not access private fields via reflection in PathPlannerTrajectory.");
            e.printStackTrace(System.err);
        }
    }


    public static boolean shouldFlip() {
        return DriverStation.getAlliance() == Alliance.Red;
    }

    public static Pose2d flipIfShould(Pose2d pose) {
        if (shouldFlip()) {
            return allianceFlip(pose);
        }
        return pose;
    }

    public static Rotation2d flipIfShould(Rotation2d rotation2d) {
        if (shouldFlip()) {
            return allianceFlip(rotation2d);
        }
        return rotation2d;
    }

    public static Rotation2d allianceFlip(Rotation2d rotation2d) {
        return new Rotation2d(-rotation2d.getCos(), rotation2d.getSin());
    }

    public static Pose2d allianceFlip(Pose2d pose) {
        return new Pose2d(
                new Translation2d(
                        fieldLength - pose.getTranslation().getX(),
                        pose.getTranslation().getY()),
                allianceFlip(pose.getRotation()));
    }

    public static PathPlannerTrajectory allianceFlip(PathPlannerTrajectory trajectory) {
        List<Trajectory.State> newStates = new ArrayList<>();

        for (State s : trajectory.getStates()) {
            PathPlannerState state = (PathPlannerState) s;

            newStates.add(allianceFlip(state));
        }

        try {
            return constructor.newInstance(
                    newStates,
                    trajectory.getMarkers(),
                    trajectory.getStartStopEvent(),
                    trajectory.getEndStopEvent(),
                    trajectory.fromGUI);
        } catch (InstantiationException | IllegalAccessException | IllegalArgumentException
                | InvocationTargetException e) {
            return new PathPlannerTrajectory();
        }
    }

    public static PathPlannerState allianceFlip(PathPlannerState state) {
        PathPlannerState newState = new PathPlannerState();
        newState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
        newState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
        newState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
        newState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
        newState.holonomicRotation = Rotation2d.fromRadians(Math.PI - state.holonomicRotation.getRadians());
        newState.poseMeters = new Pose2d(FieldConstants.fieldLength - state.poseMeters.getX(), state.poseMeters.getY(), 
            Rotation2d.fromRadians(Math.PI - state.poseMeters.getRotation().getRadians()));
        newState.timeSeconds = state.timeSeconds;
        newState.velocityMetersPerSecond = state.velocityMetersPerSecond;
        return newState;
    }
}
