package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final String canbus = "Canivore";

    public static final class SwerveK {
        public static final String DB_TAB_NAME = "SwerveSubsys";

        /** Set to true to use external CANcoder for inital zero and switch to internal falcon encoder for angle control.
         * Set to false to always use external CANcoder for angle control.
         * Recommended to set to false and always use CANCoder. */
        public static final boolean kUseInternalEncoder = false;

        public static final int kPigeonCANID = 1;
        public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants kSwerveModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants in meters */
        public static final double kTrackWidth = 0.622;
        public static final double kWheelBase = 0.5207;
        public static final double kWheelCircumference = kSwerveModule.wheelCircumference;

        /*Swerve Kinematics */
        public static final Translation2d[] kModuleTranslations = {
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
        };

        public static final SwerveDriveKinematics kKinematics =
            new SwerveDriveKinematics(kModuleTranslations);

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

         /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double kOpenLoopRamp = 0.25;
        public static final double kClosedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double kAngleKP = kSwerveModule.angleKP;
        public static final double kAngleKI = kSwerveModule.angleKI;
        public static final double kAngleKD = kSwerveModule.angleKD;
        public static final double kAngleKF = kSwerveModule.angleKF;

        /* Drive Motor PID Values */
        public static final double kDriveKP = 0.10;
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double kDriveKS = 0.3 / 12; //TODO: This must be tuned to specific robot
        public static final double kDriveKV = 1.0 / 12;
        public static final double kDriveKA = 0.2 / 12;

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
        public static final double kMaxVelocityMps = 4.5; //TODO: This must be tuned to specific robot
        /* Radians per Second */
        public static final double kMaxAngularVelocityRadps = 11.5; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(3.4) ;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(147.65);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(73.38);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back right Module - Module 3 */
        public static final class Mod3 {


            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(39.375) ;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
   }

    // Elevator tilting motor
    public static final class TiltK {
        public static final String DB_TAB_NAME = "TiltSubsys";
        //199.83 start tilt
        
        public static final int kTiltMotorCANID = 13;
        public static final int kTiltQuadratureEncoderA = 6;
        public static final int kTiltQuadratureEncoderB = 7;
        public static final int kTiltAbsoluteEncoderPort = 8;
        public static final int kTiltMotorCurrLimit = 20;

        public static final double kAbsZeroDegreeOffset = 199.8;    //where zero is at
        public static final double kAbsMaxDegree = 33; //max possible from offset

        public static final double kMaxAngleDegrees = 45;
        public static final double kMinAngleDegrees = 0;
        public static final double kMaxVelocity = 1.0; //meters per sec
        public static final double kMaxAcceleration = 1.0; //meters per sec squared
        public static final double kP = 0.25;
        public static final double kD = 0.01;
        public static final double kS = 1.2;
        public static final double kGearRatio = 25.0 / 1.0;
        public static final DCMotor kMotor = DCMotor.getFalcon500(1);
        public static final double kV = kMotor.KvRadPerSecPerVolt / kGearRatio;
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(kS, kV);

    }

    // Elevator lifting motor(s)
    public static final class ElevatorK {
        public static final String DB_TAB_NAME = "LiftSubsys";

        // max height 130580
        public static final int kLeftElevatorCANID = 11;
        public static final int kRightElevatorCANID = 12;
        public static final int kLowerLimitSwitch = 5;

        public static final int kLowerLimitSwitchPort = 5;

         /* Elevator Current Limiting */
         public static final int kElevatorContinuousCurrentLimit = 10;
         public static final int kElevatorPeakCurrentLimit = 10;
         public static final double kElevatorePeakCurrentDuration = 0.1;
         public static final boolean kElevatorEnableCurrentLimit = true;
         public static final int kElevatorForwardLimit = 130000;
         public static final int kElevatorReverseLimit = 0;
         public static final double kMaxTiltMinTicks = 12259.0;
         public static final double kMaxTiltMinHeight = 0; //TODO: Convert above to height
         public static final boolean kElevatorEnableForwardLimit = true;
         public static final boolean kElevatorEnableReverseLimit = true;

        public static final double kGearRatio = 25.0 / 1.0;
        public static final DCMotor kMotor = DCMotor.getFalcon500(1);
        public static final double kElevatorP = 0.25;
        public static final double kElevatorD = 0.01;
        public static final double kElevatorS = 1.2;
        public static final double kElevatorV = kMotor.KvRadPerSecPerVolt / kGearRatio;
        public static final double kDrumRadiusMeters = Units.inchesToMeters(1.8);
        public static final double kDrumCircumferenceMeters = kDrumRadiusMeters * 2 * Math.PI;
        public static final double kCarriageMassKg = Units.lbsToKilograms(50);
        public static final double kMinHeightMeters = Units.inchesToMeters(0);
        public static final double kMaxHeightMeters = Units.inchesToMeters(50);   //assuming 0 @ lowest
        public static final double kSafeHeight = Units.inchesToMeters(0);   //where wrist is free to move

        public static final double kMaxVelocity = 1.0; // Meters Per Second
        public static final double kMaxAcceleration = 1.0; // Meters Per Second Squared

        public static final ElevatorFeedforward kFeedforward = new ElevatorFeedforward(kElevatorS, kCarriageMassKg, kElevatorV);
        public static final TrapezoidProfile.Constraints kConstraints =
            new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 8; 
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        //weight for trusting vision over odometry (higher value = less trust) currently unused
        public static final Matrix<N3, N1> kVisionStdDevs_DefaultTrust = VecBuilder.fill(0.9, 0.9, 0.9);
        public static final Matrix<N3, N1> kVisionStdDevs_NoTrust = VecBuilder.fill(100, 100, 100);
        
        public static double kPXController = 8;
        public static double kPYController = 8;
        public static double kPThetaController = 6.75;
        public static final double kDThetaController = 0.1;
        public static final double kFThetaController = 1;

        public static final double kOffBalanceAngleThresholdDegrees = Math.toRadians(10);
        public static final double kOnBalanceAngleThresholdDegrees  = Math.toRadians(5);
        public static final double kMinimumBalanceDegrees = 2;

        public static final double kAlignAngleThresholdRadians = Math.toRadians(2.5);

        // Constraint for the motion profiled robt angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        
        //used for PPSwerve Auto Builder
        public static final PIDConstants kTranslationPID = new PIDConstants(kPXController, 0 , 0);   //x & y
        public static final PIDConstants kRotationPID = new PIDConstants(kPThetaController, 0, kDThetaController);
    }

    //in meters
    public static final class VisionConstants {
        public static final double kCameraHeight = 0.88265;
        public static final double kCameraX = 0;
        public static final double kCameraY = 0.1524;
        public static final double kTargetHeight = 1;   //TODO: update value
    }

    public static final class WristK {
        public static final String DB_TAB_NAME = "WristSubsys";

        //lower: .527
        public static final int kWristCANID = 21;

        public static final int kWristCurrLimit = 35;
        public static final double kAbsEncoderTicksPerRotation = 1024; // change later
        public static final double kP = 0.6;
        public static final double kD = 0.05;
        public static final double kS = 1.2;    // change values later
        public static final double kMaxVelocity = 0.5;
        public static final double kMaxAcceleration = 1; // change later
        public static final TrapezoidProfile.Constraints kConstraints =
            new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
        public static final double kZeroDegOffset = 5.5;
        public static final double kMinAnglePosition = 0.01;
        public static final double kMaxAnglePosition = 0.2903;
        public static final double kMinAngleDegrees = -37.5;
        public static final double kMaxAngleDegrees = 82;
        public static final double kGearRatio = (80.0/1) / (16.0/22.0); // change later
        public static final double kDrumRadiusMeters = Units.inchesToMeters(2); // change later
        public static final double kDrumCircumferenceMeters = kDrumRadiusMeters * 2 * Math.PI;
        public static final double KvRadPerSecPerVolt = (DCMotor.getNEO(1).KvRadPerSecPerVolt / kGearRatio) + 0; // 0 for later fudge factor


        public static final ArmFeedforward kFeedforward = new ArmFeedforward(0.1, 0, KvRadPerSecPerVolt);
        // public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.5, DCMotor.getNEO(1).KvRadPerSecPerVolt * kGearRatio);
    }

    public static final class TheClawK{
        public static final String DB_TAB_NAME = "ClawSubsys";

        public static final int kTheClawID = 0; 
        public static final int kLeftEyeID = 0;
        public static final int kRightEyeID = 1;
    }

    public static final class IndicatorLightsK{
        public static final String DB_TAB_NAME = "LEDSubsys";

        public static final int kNumLEDs = 20; //TODO: change this
        public static final int kLedPort = 0; //TODO: change this
    }
}
