package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
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


        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // Current ChassisSpeeds supplier
            this::setChassisSpeeds, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
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

    public void resetPose(Pose2d pose) {
        kOdometry.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            backLeft.getPosition(),
            backRight.getPosition(),
            frontLeft.getPosition(),
            frontRight.getPosition()
          }, pose);
    }
// Added for Auton
    public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      // supplier for chassisSpeed, order of motors need to be the same as the consumer of ChassisSpeed
      frontLeft.getState(), 
      backLeft.getState(),
      frontRight.getState(),
      backRight.getState()
      );
  }
  // Added for Auton

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
      DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }
    @Override
    public void periodic() {
        kOdometry.update(getRotation2d(), new SwerveModulePosition[] {
            backLeft.getPosition(),
            backRight.getPosition(),
            frontLeft.getPosition(),
            frontRight.getPosition()
          });
        // odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
       // SmartDashboard.putNumber("Robot Heading", gyro.getAngle(gyro.getYawAxis()));
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
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