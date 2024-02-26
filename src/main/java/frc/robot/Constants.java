package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters =  0.1; // the mk4i wheels are 10cm diameter
        public static final double kDriveMotorGearRatio = 1 / 6.75; // This is the L2 ratio of our mk4i swerves
        public static final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0); // The steering gear ratio of the mk4i is 150/7:1, or 21.432857... 
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.2;
    }

    public static final class DriveConstants {
        public static final boolean kGyroReversed = true; // this might have fixed  our headless mode the other day. JD 2-21-24
        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( 
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // +y,-x Front Left
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // +y,+x Front Right
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // -y,-x Rear Left
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // -y,+x Rear Right

        public static final int kFrontLeftDriveMotorPort = 42;
        public static final int kBackLeftDriveMotorPort = 32;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kBackRightDriveMotorPort = 22;

        public static final int kFrontLeftTurningMotorPort = 41;
        public static final int kBackLeftTurningMotorPort = 31;
        public static final int kFrontRightTurningMotorPort = 11;
        public static final int kBackRightTurningMotorPort = 21;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftSwerveAbsoluteEncoderPort = 40;
        public static final int kBackLeftSwerveAbsoluteEncoderPort = 30;
        public static final int kFrontRightSwerveAbsoluteEncoderPort = 10;
        public static final int kBackRightSwerveAbsoluteEncoderPort = 20;

        public static final boolean kFrontLeftSwerveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftSwerveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightSwerveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightSwerveAbsoluteEncoderReversed = true;

        // We don't need to configure offsets in here because our cancoders have been zero'd through phoenix tuner. 
        // if we NEEDED to reset them within the code without access to phoenix tuner,
        // we could physically realign them to zero north-south, and uncomment the line "absoluteEncoder.setPosition(0);" in swervemodule.java. 
        public static final double kFrontLeftSwerveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kBackLeftSwerveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kFrontRightSwerveAbsoluteEncoderOffsetRad = 0.0;
        public static final double kBackRightSwerveAbsoluteEncoderOffsetRad = 0.0;
        // - our actual offsets zero'd in phoenix tuner.
        // public static final double kFrontLeftSwerveAbsoluteEncoderOffsetRad = -0.374267578125;
        // public static final double kBackLeftSwerveAbsoluteEncoderOffsetRad = -0.283203125;
        // public static final double kFrontRightSwerveAbsoluteEncoderOffsetRad = 0.218994140625;
        // public static final double kBackRightSwerveAbsoluteEncoderOffsetRad = 0.12646484375;


        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.8768; //I don't know how fast these can actually go. faster than this. 
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        // set your safe speeds here. Perhaps we should put these on an option in constants, or even a button switch. 
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond /2 ; // 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond /2 ;// 6;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;//2; // 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;//2  // 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;// / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;// 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 2; // Math.PI / 4;
        public static final double kPXController = 0.1;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 0.1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 2;

        public static final int kDriverYAxis = 1; // -forward and +reverse
        public static final int kDriverXAxis = 0; // -left and +right
        public static final int kDriverRotAxis = 4; // 4 is the "rx" axis. Right stick, X. 
        public static final int kDriverFieldOrientedButtonIdx = 5; // lets try using the "L" button for this. 

        public static final double kDeadband = 0.05;

        public static final int kOperatorGreenButton = 8;
        
    }

    public static final class IntakeConstants {
        public static final int kMotorPort = 52;
        public static final double kSpeed = 0.05;
        public static int kSensorPort = 0; // DIO
    }

    public static final class ShooterConstants {
        public static final int kMotorPort = 51;
        public static final double kSpeed = 0.05;
    }
}