package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {

        /** Set to true to use external CANcoder for inital zero and switch to internal falcon encoder for angle control.
         * Set to false to always use external CANcoder for angle control.
         * Recommended to set to false and always use CANCoder. */
        public static final boolean useInternalEncoder = false;

        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants in meters */
        public static final double trackWidth = 0.622;
        public static final double wheelBase = 0.5207;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*Swerve Kinematics */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

         /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /* Radians per Second */
        public static final double maxAngularVelocity = 11.5; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(182.37) ;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(328.5);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }
        /* Back left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(252);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90.17) ;
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        // /* Back left Module - Module 3 */
        // public static final class Mod3 {
        //     public static final int driveMotorID = 6;
        //     public static final int angleMotorID = 5;
        //     public static final int canCoderID = 5;
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(254) ;
        //     public static final SwerveModuleConstants constants =
        //             new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        // }

        // /* Back right Module - Module 4 */
        // public static final class Mod2 {
        //     public static final int driveMotorID = 8;
        //     public static final int angleMotorID = 7;
        //     public static final int canCoderID = 7;
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(88);
        //     public static final SwerveModuleConstants constants =
        //             new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        // }

    }

    // Elevator tilting motor
    public static final class ElevatorTiltK {
        public static final int kCANID = 11;

        public static final double kMaxAngleDegrees = 30;
        public static final double kMinAngleDegrees = 0;
        public static final double kMaxVelocity = 1.0; //meters per sec
        public static final double kMaxAcceleration = 1.0; //meters per sec squared
        public static final double kP = 0.25;
        public static final double kD = 0.01;
        public static final double kS = 1.2;
        public static final double kGearRatio = 49.0 / 1.0;
        public static final DCMotor kMotor = DCMotor.getNeo550(1);
        public static final double kV = kMotor.KvRadPerSecPerVolt / kGearRatio;
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(kS, kV);

        public static final double kArmLengthMeters = 1;
        public static final double kJKgMetersSquared = 1;
        public static final double kMinAngleRads = Units.degreesToRadians(kMinAngleDegrees);
        public static final double kMaxAngleRads = Units.degreesToRadians(kMaxAngleDegrees);
        public static final double kArmMassKg = 10;

        public static final int kAngleAbsPort = 0;
        public static final int kAngleRelAPort = 1;
        public static final int kAngleRelBPort = 2;
    }

    // Elevator lifting motor(s)
    public static final class ElevatorLiftK {
        // Motor Constants
        public static final int kCANID = 10;

        public static final double kGearRatio = 25.0 / 1.0;
        public static final DCMotor kMotor = DCMotor.getFalcon500(1);
        public static final double kP = 0.25;
        public static final double kD = 0.01;
        public static final double kS = 1.2;
        public static final double kV = kMotor.KvRadPerSecPerVolt / kGearRatio;
        public static final double kDrumRadiusMeters = Units.inchesToMeters(2);
        public static final double kDrumCircumferenceMeters = kDrumRadiusMeters * 2 * Math.PI;
        public static final double kCarriageMassKg = Units.lbsToKilograms(50);
        public static final double kMinHeightMeters = Units.inchesToMeters(0);
        public static final double kMaxHeightMeters = Units.inchesToMeters(50);

        public static final double kMaxTicks = 139000;

        public static final double kMaxVelocity = 1.0; // Meters Per Second
        public static final double kMaxAcceleration = 1.0; // Meters Per Second Squared

        public static final ElevatorFeedforward kFeedforward = new ElevatorFeedforward(kS, kCarriageMassKg, kV);
        public static final TrapezoidProfile.Constraints kConstraints =
            new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    //in meters
    public static final class VisionConstants{
        public static final double kCameraHeight = 0.88265;
        public static final double kCameraX = 0;
        public static final double kCameraY = 0.1524;
    }

    public static final class SmartDashboardKeys{
        public static final String kPhotonDistanceKey = "Photon Distance Meters";
        public static final String kLeftFrontDriveTemp = "Left Front Drive Temp";
        public static final String kLeftFrontAngleTemp = "Left Front Angle Temp";
        public static final String kRightFrontDriveTemp = "Right Front Drive Temp";
        public final String kRightFrontAngleTemp = "Right Front Angle Temp";
        public final String kLeftRearDriveTemp = "Left Rear Drive Temp";
        public final String kLeftRearAngleTemp = "Left Rear Angle Temp";
        public final String kRightRearDriveTemp = "Right Rear Drive Temp";
        public final String kRightRearAngleTemp = "Right Rear Angle Temp";
        
    }

}
