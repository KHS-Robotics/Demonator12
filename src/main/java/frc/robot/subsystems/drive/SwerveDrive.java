/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.Field;
import frc.robot.RobotContainer;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive extends SubsystemBase {
  public static double kMaxSpeedMetersPerSecond = 3.5;
  public static double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
  public static double kMaxAcceleration = 99;
  public static double kMaxDeceleration = -8; // this is based on nothing at all
  public static double offset;
  public Pose2d startingPose;
  public double angleSetpoint;
  private PIDController targetPid;
  private final Translation2d frontLeftLocation = new Translation2d(0.2984, 0.2984);
  private final Translation2d frontRightLocation = new Translation2d(0.2984, -0.2984);
  private final Translation2d rearLeftLocation = new Translation2d(-0.2984, 0.2984);
  private final Translation2d rearRightLocation = new Translation2d(-0.2984, -0.2984);

  private SlewRateLimiter xRateLimiter;
  private SlewRateLimiter yRateLimiter;

  public double currentX, currentY;

  public boolean isCalibrated = false;
  private boolean loggedPoseError = false;

  public static final SwerveModule frontLeft = new SwerveModule(
      "FL",
      RobotMap.FRONT_LEFT_DRIVE,
      RobotMap.FRONT_LEFT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.FRONT_LEFT_PIVOT_ENCODER,
      225);
  public static final SwerveModule frontRight = new SwerveModule(
      "FR",
      RobotMap.FRONT_RIGHT_DRIVE,
      RobotMap.FRONT_RIGHT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.FRONT_RIGHT_PIVOT_ENCODER,
      135);
  public static final SwerveModule rearLeft = new SwerveModule(
      "RL",
      RobotMap.REAR_LEFT_DRIVE,
      RobotMap.REAR_LEFT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.REAR_LEFT_PIVOT_ENCODER,
      315);
  public static final SwerveModule rearRight = new SwerveModule(
      "RR",
      RobotMap.REAR_RIGHT_DRIVE,
      RobotMap.REAR_RIGHT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.REAR_RIGHT_PIVOT_ENCODER,
      45);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation,
      frontRightLocation, rearLeftLocation, rearRightLocation);

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics,
      RobotContainer.navx.getRotation2d(), new SwerveModulePosition[] {
          new SwerveModulePosition(0, new Rotation2d(frontLeft.getAngle())),
          new SwerveModulePosition(0, new Rotation2d(frontRight.getAngle())),
          new SwerveModulePosition(0, new Rotation2d(rearLeft.getAngle())),
          new SwerveModulePosition(0, new Rotation2d(rearRight.getAngle()))
      }, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

  // private final SwerveDriveOdometry odometry = new
  // SwerveDriveOdometry(kinematics, this.getAngle());

  /**
   * Constructs Swerve Drive
   */
  public SwerveDrive() {

    targetPid = new PIDController(Constants.TARGET_P, Constants.TARGET_I, Constants.TARGET_D);
    targetPid.enableContinuousInput(-180.0, 180.0);
    targetPid.setTolerance(1);

    xRateLimiter = new SlewRateLimiter(kMaxAcceleration, kMaxDeceleration, 0);
    yRateLimiter = new SlewRateLimiter(kMaxAcceleration, kMaxDeceleration, 0);
  }

  /**
   * Returns the angle of the robot as a Rotation2d as read by the navx.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees((-RobotContainer.navx.getAngle() + offset) % 360);
  }

  public double getYaw() {
    return normalizeAngle(-RobotContainer.navx.getYaw() + offset);
  }

  public void setOffset(double offset) {
    SwerveDrive.offset = offset;
  }

  public double sensControl(double var) {
    return Constants.SENS * Math.pow(var, 3) + (1 - Constants.SENS) * var;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed        Speed of the robot in the y direction (sideways) in m/s.
   * @param rot           Angular rate of the robot in rad/sec.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (Math.abs(rot) < 0.005 && Math.abs(xSpeed) < 0.015 && Math.abs(ySpeed) < 0.015) {
      frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(frontLeft.getAngle())));
      frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(frontRight.getAngle())));
      rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rearLeft.getAngle())));
      rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rearRight.getAngle())));
    } else {
      var swerveModuleStates = kinematics
          .toSwerveModuleStates(
              fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xRateLimiter.calculate(xSpeed), yRateLimiter.calculate(ySpeed), rot, getPose().getRotation())
                  : new ChassisSpeeds(xRateLimiter.calculate(xSpeed), yRateLimiter.calculate(ySpeed), rot));

      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      rearLeft.setDesiredState(swerveModuleStates[2]);
      rearRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0], true);
    frontRight.setDesiredState(desiredStates[1], true);
    rearLeft.setDesiredState(desiredStates[2], true);
    rearRight.setDesiredState(desiredStates[3], true);
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    setModuleStates(kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, poseEstimator.getEstimatedPosition().getRotation())));
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(), pose);
  }

  public void setPID(double p, double i, double d) {
    targetPid.setPID(p, i, d);
  }

  public void rotateToAngleInPlace(double setAngle) {
    holdAngleWhileDriving(0, 0, Rotation2d.fromDegrees(setAngle), false);
  }

  public void holdAngleWhileDriving(double x, double y, Rotation2d setAngle, boolean fieldOriented) {
    var rotateOutput = MathUtil.clamp(targetPid.calculate(
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(), normalizeAngle(setAngle.getDegrees())), -1, 1)
        * kMaxAngularSpeedRadiansPerSecond;
    this.drive(x, y, rotateOutput, fieldOriented);
  }

  public void holdAngleWhileDriving(double x, double y, double setAngle, boolean fieldOriented) {
    holdAngleWhileDriving(x, y, Rotation2d.fromDegrees(setAngle), fieldOriented);
  }

  public boolean atSetpoint() {
    return targetPid.atSetpoint();
  }

  public boolean atSetpoint(double allowableError) {
    targetPid.setTolerance(allowableError);
    return targetPid.atSetpoint();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Updates the field relative position of the robot.
   * 
   */
  public void updateOdometry() {
    var hasValidMeasurements = true;
    var modulePositions = getSwerveModulePositions();
    for (var i = 0; i < modulePositions.length; i++) {
      var pos = modulePositions[i];
      if (Double.isNaN(pos.distanceMeters) || Double.isNaN(pos.angle.getRadians())) {
        DriverStation.reportError("Rejected module state! (index=" + i + ")", false);
        hasValidMeasurements = false;
        break;
      }
    }

    if (hasValidMeasurements) {
      poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getAngle(), modulePositions);
    }

    var pose = poseEstimator.getEstimatedPosition();

    if (!this.loggedPoseError && (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()))) {
      this.loggedPoseError = true;
      for (var pos : modulePositions) {
        DriverStation.reportError("Bad module state! Check output for details.", false);
        System.err.println("Bad module states: " + pos);
      }
    }

  }

  public void resetOdometry() {
    RobotContainer.navx.reset();
    offset = Math.toDegrees(Math.PI);
    poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(),
        new Pose2d(this.getPose().getX(), this.getPose().getY(), new Rotation2d(Math.PI)));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public double getHeading() {
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    double headingRR = Math.toDegrees(Math.atan2(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
    double headingFR = headingRR + getPose().getRotation().getDegrees();
    return headingFR;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState()
    };
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
    };
  }

  public void stop() {
    frontRight.stop();
    frontLeft.stop();
    rearRight.stop();
    rearLeft.stop();
  }

  public void lock() {
    frontRight.setDesiredState(0, 45);
    frontLeft.setDesiredState(0, -45);
    rearRight.setDesiredState(0, -45);
    rearLeft.setDesiredState(0, 45);
  }

  public void resetNavx() {
    resetNavx(getPose());
  }

  public void resetNavx(Pose2d currentPose) {
    targetPid.reset();
    offset = (currentPose.getRotation().getDegrees() + 180) % 360;
    RobotContainer.navx.reset();
    startingPose = currentPose;
  }

  public void resetPid() {
    targetPid.reset();
  }

  public boolean facingInfield() {
    return (180 - Math.abs(getYaw())) < 10 || Math.abs(getYaw()) < 10;
  }

  private static double normalizeAngle(double angle) {
    if (angle > 0) {
      angle %= 360;
      if (angle > 180) {
        angle -= 360;
      }
    } else if (angle < 0) {
      angle %= -360;
      if (angle < -180) {
        angle += 360;
      }
    }
    return angle;
  }

  public void logTargetChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SmartDashboard.putNumber("vxChassis", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("vyChassis", chassisSpeeds.vyMetersPerSecond);
  }

  public Pose2d getPoseInverted() {
    return new Pose2d(getPose().getX(), getPose().getY(), getPose().getRotation().plus(new Rotation2d(Math.PI)));
  }

  @Override
  public void periodic() {
    updateOdometry();
    RobotContainer.field.setRobotPose(Field.flipPose(poseEstimator.getEstimatedPosition()));
    SmartDashboard.putNumber("Pose angle", getPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("Calibrated", isCalibrated);
    logTargetChassisSpeeds(getChassisSpeeds());
  }

}