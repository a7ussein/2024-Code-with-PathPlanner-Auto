package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftSwerveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftSwerveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftSwerveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightSwerveAbsoluteEncoderPort,
            DriveConstants.kFrontRightSwerveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightSwerveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftSwerveAbsoluteEncoderPort,
            DriveConstants.kBackLeftSwerveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftSwerveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightSwerveAbsoluteEncoderPort,
            DriveConstants.kBackRightSwerveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightSwerveAbsoluteEncoderReversed);

    private ADIS16470_IMU gyro = new ADIS16470_IMU();
    
    public final SwerveDriveOdometry kOdometry = new SwerveDriveOdometry(
            Constants.DriveConstants.kDriveKinematics, getRotation2d(),
            new SwerveModulePosition[] {
              backLeft.getPosition(),
              backRight.getPosition(),
              frontLeft.getPosition(),
              frontRight.getPosition()
            }, new Pose2d(0, 0, new Rotation2d()));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void resetTurningEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading() {
        System.out.println("Gyro Reset");
        gyro.setGyroAngleZ(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(DriveConstants.kGyroReversed ? gyro.getAngle(gyro.getYawAxis()) * -1 : gyro.getAngle(gyro.getYawAxis()), 360);
    }
    
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return kOdometry.getPoseMeters();
    }

    // called from auto
    public void resetOdometry(Pose2d pose) {
        Rotation2d rot = Rotation2d.fromDegrees(180);
        kOdometry.resetPosition(rot, new SwerveModulePosition[] {
            backLeft.getPosition(),
            backRight.getPosition(),
            frontLeft.getPosition(),
            frontRight.getPosition()
          }, pose);
    }


    @Override
    public void periodic() {
        kOdometry.update(getRotation2d(), new SwerveModulePosition[] {
            backLeft.getPosition(),
            backRight.getPosition(),
            frontLeft.getPosition(),
            frontRight.getPosition()
          });
          SmartDashboard.putNumber("Robot Heading", getHeading());
          SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}