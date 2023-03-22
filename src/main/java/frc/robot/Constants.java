package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.LedUtils;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final String canbus = "Canivore";
    
    public static final class AtHome {
        public static final boolean AreWe = false;
        public static final double chargeStationApproachPwr = 2.25;
        public static final double forwardPitch = 3;
        public static final double reversePitch = 6;
        public static final double forwardPwr = 2;
        public static final double reversePwr = 2;
    }

    public static final class AtComp{
        public static final double chargeStationApproachPwr = 3;
        public static final double forwardPitch = 4.5;
        public static final double reversePitch = 4.5;
        public static final double forwardPwr = 2.15;
       public static final double reversePwr = 2.15;
    }

    public static final class SwerveK {
        public static final String DB_TAB_NAME = "SwerveSubsys";

        /**
         * Set to true to use external CANcoder for inital zero and switch to internal
         * falcon encoder for angle control.
         * Set to false to always use external CANcoder for angle control.
         * Recommended to set to false and always use CANCoder.
         */
        public static final boolean kUseInternalEncoder = false;

        public static final int kPigeonCANID = 1;
        public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants kSwerveModule = COTSFalconSwerveConstants
                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants in meters */
        public static final double kTrackWidth = Units.inchesToMeters(27);
        public static final double kWheelBase = Units.inchesToMeters(27);
        public static final double kWheelCircumference = kSwerveModule.wheelCircumference;

        /* Swerve Kinematics */
        public static final Translation2d[] kModuleTranslations = {
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
        };

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);

        /* Module Gear Ratios */
        public static final double kDriveGearRatio = kSwerveModule.driveGearRatio;
        public static final double kAngleGearRatio = kSwerveModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean kInvertAngleMotor = kSwerveModule.angleMotorInvert;
        public static final boolean kInvertDriveMotor = kSwerveModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean kInvertCanCoder = kSwerveModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int kAngleContinuousCurrentLimit = 25;
        public static final int kAnglePeakCurrentLimit = 40;
        public static final double kAnglePeakCurrentDuration = 0.1;
        public static final boolean kAngleEnableCurrentLimit = true;

        public static final int kDriveContinuousCurrentLimit = 35;
        public static final int kDrivePeakCurrentLimit = 60;
        public static final double kDrivePeakCurrentDuration = 0.1;
        public static final boolean kDriveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double kOpenLoopRamp = 0.25;
        public static final double kClosedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double kAngleKP = kSwerveModule.angleKP;
        public static final double kAngleKI = kSwerveModule.angleKI;
        public static final double kAngleKD = kSwerveModule.angleKD;
        public static final double kAngleKF = kSwerveModule.angleKF;

        /* Drive Motor PID Values */
        public static final double kDriveKP = 0.05;
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double kDriveKS = 0.32 / 12; // TODO: This must be tuned to specific robot
        public static final double kDriveKV = 1.51 / 12;
        public static final double kDriveKA = 0.27 / 12;

        /* Feedforwards */
        public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward( // real
                kDriveKS, // Voltage to break static friction
                kDriveKV, // Volts per meter per second
                kDriveKA // Volts per meter per second squared
        );
        // Steer feed forward
        public static final SimpleMotorFeedforward kSteerFF = new SimpleMotorFeedforward( // real
                0.5, // Voltage to break static friction
                0.23, // Volts per radian per second
                0.0056 // Volts per radian per second squared
        );

        /* Swerve Profiling Values */
        public static final double kMaxVelocityMps = 4.5; // TODO: This must be tuned to specific robot //4.5
        /* Radians per Second */
        public static final double kMaxAngularVelocityRadps = 10.0; // TODO: This must be tuned to specific robot    //11.5

        /* Neutral Modes */
        public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(3.95);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(147.8);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(73.4);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back right Module - Module 3 */
        public static final class Mod3 {

            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(42.36);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3.75;
        public static final double kMaxAccelerationMetersPerSecondSquared = 6;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 3*Math.PI;

        // weight for trusting vision over odometry (higher value = less trust)
        // currently unused
        public static final Matrix<N3, N1> kOdoStdDevs_DefaultTrust = VecBuilder.fill(0.1, 0.1, 0.025);
        public static final Matrix<N3, N1> kVisionStdDevs_DefaultTrust = VecBuilder.fill(0.9, 0.9, 10);
        
        public static final Matrix<N3, N1> kVisionStdDevs_NoTrust = VecBuilder.fill(100, 100, 100);

        public static double kPXController = 8; // 8
        public static double kPYController = 8; // 8
        public static double kPThetaController = 5; // 1
        public static final double kDThetaController = 0.1;
        public static final double kFThetaControllerAuto = 0;
        public static final double kFThetaController = 1;

        public static final double kOffBalanceAngleThresholdDegrees = Math.toRadians(10);
        public static final double kOnBalanceAngleThresholdDegrees = Math.toRadians(5);
        public static final double kMinimumBalanceDegrees = 2;

public static final double kAlignAngleThresholdRadians = Math.toRadians(2.5);

        // Constraint for the motion profiled robt angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        // used for PPSwerve Auto Builder
        public static final PIDConstants kTranslationPID = new PIDConstants(kPXController, 0, 0); // x & y
        public static final PIDConstants kRotationPID = new PIDConstants(kPThetaController, 0, kDThetaController);

    }

//     public static final class FieldK {
//         public static final Pose3d kRedAllianceOrigin = new Pose3d(Units.inchesToMeters(651.25), 0.0, 0.0, new Rotation3d(0, 0, Math.PI));
//     }
    
    // Elevator tilting motor
    public static final class TiltK {
        public static final String DB_TAB_NAME = "TiltSubsys";
        // 199.83 start tilt

        public static final int kMotorCANID = 13;
        public static final int kQuadEncoderA = 6;
        public static final int kQuadEncoderB = 7;
        public static final int kAbsoluteEncoderPort = 8;
        public static final int kMotorCurrLimit = 20;
        public static final int kHomeSwitchPort = 9;
        public static final int kDiskBrakePort = 15;

        public static final double kBeforeBrakeTime = .25;     //sec
        public static final double kAfterBrakeTime = .25;
        public static final double kTeleopBrakeTime = 1.5;

        public static final double kAbsZeroDegreeOffset = 199.8; // where zero is at
        public static final double kAbsMaxDegree = 30; // max possible from offset

        
        public static final double kTopAngleDegrees = 15;
        public static final double kTopConeAngleDegrees = 29.1;
        public static final double kTopCubeAngleDegrees = 29.1;
        public static final double kMidConeAngleDegrees = 22.118;
        public static final double kMidCubeAngleDegrees = 22.041;
        public static final double kMidAngleDegrees = 29.1;
        public static final double kBotAngleDegrees = 0;
        public static final double kSubstationAngleDegrees = 0;
        public static final double kExtendedSubstationAngleDegrees = 18;
        public static final double kMinAngleDegrees = 0;

        public static final double kMaxAngleDegrees = 29.1;
        public static final double kMaxVelocity = 180; // degrees per sec
        public static final double kMaxAcceleration = 200.0; // degrees per sec squared
        public static final double kMaxVelocityForward = kMaxVelocity * .75;
        public static final double kMaxAccelerationForward = kMaxAcceleration * .75;

        public static final double kP = .75;
        public static final double kD = 0.0;
        public static final double kS = 1.5;
        public static final double kGearRatio = ((49.0 / 1.0) * (37.0 / 21.0));
        public static final DCMotor kMotor = DCMotor.getFalcon500(1);
        public static final double kV = kMotor.KvRadPerSecPerVolt / kGearRatio;
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(kS,
                Units.radiansToDegrees(kV));

        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVelocity,
                kMaxAcceleration);

    }

    // Elevator lifting motor(s)
    public static final class ElevatorK {
        public static final String DB_TAB_NAME = "LiftSubsys";

        // max height 130580
        public static final int kLeftCANID = 11;
        public static final int kRightCANID = 12;
        public static final int kLowerLimitSwitch = 5;

        public static final int kLowerLimitSwitchPort = 5;

        /* Elevator Current Limiting */
        public static final double kVoltageCompSaturationVolts = 12.0;
        public static final int kContinuousCurrentLimit = 8;
        public static final int kPeakCurrentLimit = 20;
        public static final double kPeakCurrentDuration = 0.1;
        public static final boolean kEnableCurrentLimit = true;
        public static final int kForwardLimit = 130000;
        public static final int kReverseLimit = 0;
        public static final double kMaxTiltMinTicks = 12259.0; // raw height when fully tilted
        public static final double kMaxTiltMinHeight = 0; // TODO: Convert above to height
        public static final boolean kEnableForwardLimit = true;
        public static final boolean kEnableReverseLimit = true;

        public static final double kGearRatio = 12.0 / 1.0;
        public static final DCMotor kMotor = DCMotor.getFalcon500(1);
        public static final double kP = 22; //sysid 9.2597E-05
        public static final double kD = 0;
        public static final double kS = 0.16114;
        public static final double kV = 9.7833;
        public static final double kA = 0.4375; // 0.41885 from Tyler
        public static final double kG = 0.1744;
        
        public static final double kPHold = .7;
        public static final double kDHold = 0;
        public static final double kHoldKs = .705;

        public static final double kDrumRadiusMeters = Units.inchesToMeters(0.8459);
        public static final double kDrumCircumferenceMeters = kDrumRadiusMeters * 2 * Math.PI;
        public static final double kElevatorHeightOffset = 0.015; // offset in meters
        public static final double kCarriageMassKg = Units.lbsToKilograms(40);
        public static final double kMinHeightMeters = Units.inchesToMeters(0);
        public static final double kMaxHeightMeters = Units.inchesToMeters(50); // assuming 0 @ lowest
        public static final double kSafeHeight = kElevatorHeightOffset; // where wrist is free to move

        public static final double kTopHeightMeters = Units.inchesToMeters(41); // TODO: change later :DDD
        public static final double kTopCubeHeightM = 0.615;
        public static final double kTopConeHeightM = 0.68;
        public static final double kMidConeHeightM = 0.42;
        public static final double kMidCubeHeightM = 0.36;
        public static final double kMidHeightMeters = Units.inchesToMeters(30); // TODO: change later :DDD
        public static final double kBotHeightMeters = 0; //TODO: change later :DDD
        public static final double kSubstationHeightM = 0.4095;
        public static final double kExtendedSubstationHeightM = .46;
        public static final double kSubstationConeHeightM = 0;

        public static final double kMaxVelocity = 2.75; // Meters Per Second
        public static final double kMaxAcceleration = 2.60; // Meters Per Second Squared
        public static final double kMaxAccelerationDown = 1.85;

        public static final ElevatorFeedforward kFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVelocity,
                kMaxAcceleration);      
                public static final TrapezoidProfile.Constraints kConstraintsDown = new TrapezoidProfile.Constraints(kMaxVelocity,
                kMaxAccelerationDown);      
    }

    public static final class WristK {
        public static final String DB_TAB_NAME = "WristSubsys";
        public static final int kCANID = 21;

        public static final int kCurrLimit = 20;
        public static final double kAbsEncoderTicksPerRotation = 1024;
        public static final double kP = .11;
        public static final double kD = 0.00;
        public static final double kS = 1.5; 
        public static final double kHoldKs = .15;
        public static final double kMaxVelocity = 12000; // deg/sec
        public static final double kMaxAcceleration = 14000; // deg/sec^2

        public static final double kZeroDegOffset = 5.5;
        public static final double kMinDeg = -35;
        public static final double kMaxDeg = 78;

        public static final double kTopConeDeg = 40.5;
        public static final double kTopCubeDeg = 29.888;
        public static final double kMidConeDeg = 36.0;
        public static final double kMidCubeDeg = 25.316;
        public static final double kPickupDeg = -8.5;
        public static final double kSubstationDeg = 0;
        public static final double kExtendedSubstationDeg = 15;
        
        public static final double kGearRatio = (80.0 / 1) / (16.0 / 22.0);
        public static final double kDrumRadiusMeters = Units.inchesToMeters(2);
        public static final double kDrumCircumferenceMeters = kDrumRadiusMeters * 2 * Math.PI;
        public static final double KvRadPerSecPerVolt = (DCMotor.getNEO(1).KvRadPerSecPerVolt / kGearRatio) + 0; // 0
                                                                                                                 // for
                                                                                                                 // later
                                                                                                                 // fudge
                                                                                                                 // factor

        public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(kMaxVelocity,
                kMaxAcceleration);
        public static final ArmFeedforward kFeedforward = new ArmFeedforward(0.1, 0, KvRadPerSecPerVolt);
    }

    public static final class TheClawK {
        public static final String DB_TAB_NAME = "ClawSubsys";

        public static final int kTheID = 0;
        public static final int kClawSensor = 0;
    }

       // in meters
       public static final class VisionConstants {
        //TODO: update these values once we get them
        public static final double kCameraHeight = 0.88265;
        public static final double kCameraX = 0;
        public static final double kCameraY = 0.1524;
        public static final double kTargetHeight = 1; // TODO: update value
        public static final Pose2d kWayOutTherePose = new Pose2d(-1000, -1000, Rotation2d.fromDegrees(0));
    }

    public static final class LedK {
        public static final String DB_TAB_NAME = "LedSubsys";

        public static final int kNumLeds = 13; // two 13-led strips in parallel
        public static final int kPort = 0;

        // Base colors
        public static final Color kRed = LedUtils.fixColor(Color.kRed);
        public static final Color kGreen = LedUtils.fixColor(Color.kGreen);
        public static final Color kBlue = LedUtils.fixColor(Color.kBlue);
        public static final Color kYellow = LedUtils.fixColor(Color.kYellow);
        public static final Color kPurple = LedUtils.fixColor(Color.kPurple);

        // Specific colors
        public static final Color kCubeColor = kPurple;
        public static final Color kConeColor = kYellow;
        public static final Color kBlinkOnColor = kGreen;
        public static final Color kBlinkOffColor = Color.kBlack;

        // Behaviors
        public static final double kBlinkPeriod = 0.080;
        public static final double kBlinkCount = 10;
    }
}
