package frc.lib.util;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathPointAccessor extends PathPoint {


    public PathPointAccessor(
        Translation2d position,
        Rotation2d heading,
        Rotation2d holonomicRotation,
        double velocityOverride) {
        super(position, heading, holonomicRotation, velocityOverride);
    }

    public PathPointAccessor(
        Translation2d position,
        Rotation2d heading,
        Rotation2d holonomicRotation
    ) {
        super(position, heading, holonomicRotation);
    }


    public Translation2d getPosition() {
        return super.position;
    }

    public Rotation2d getHeading() {
        return super.heading;
    }

    public Rotation2d getHolRotation() {
        return super.holonomicRotation;
    }

    public Translation2d getVelocityOverride() {
        return super.position;
    }

    
}
