/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.pathing.PPSwerveControllerCommandReversed;
import frc.robot.subsystems.vision.PhotonWrapper;
import frc.robot.Constants;
import frc.robot.Field;
import frc.robot.RobotContainer;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive extends SubsystemBase {
  public static double kMaxSpeed = 3.5; // 3.5 meters per second
  public static double kMaxAngularSpeed = 2 * Math.PI; // pi radians per second
  public static double offset;
  public Pose2d startingPose;
  public double angleSetpoint;
  private PIDController targetPid;
  private final Translation2d frontLeftLocation = new Translation2d(0.327025, 0.2693162);
  private final Translation2d frontRightLocation = new Translation2d(0.327025, -0.2693162);
  private final Translation2d rearLeftLocation = new Translation2d(-0.327025, 0.2693162);
  private final Translation2d rearRightLocation = new Translation2d(-0.327025, -0.2693162);

  public PhotonWrapper photonCamera;

  public boolean isCalibrated = false;

  public static final SwerveModule frontLeft = new SwerveModule(
      "FL",
      RobotMap.FRONT_LEFT_DRIVE,
      RobotMap.FRONT_LEFT_PIVOT,
      Constants.FRONT_LEFT_P,
      Constants.FRONT_LEFT_I,
      Constants.FRONT_LEFT_D,
      Constants.FRONT_LEFT_KS,
      Constants.FRONT_LEFT_KV,
      Constants.FRONT_LEFT_KA,
      Constants.FRONT_LEFT_DRIVE_P,
      Constants.FRONT_LEFT_DRIVE_I,
      Constants.FRONT_LEFT_DRIVE_D,
      Constants.FRONT_LEFT_DRIVE_KS,
      Constants.FRONT_LEFT_DRIVE_KV,
      Constants.FRONT_LEFT_DRIVE_KA,
      RobotMap.FRONT_LEFT_DIGITAL_INPUT);
  public static final SwerveModule frontRight = new SwerveModule(
      "FR",
      RobotMap.FRONT_RIGHT_DRIVE,
      RobotMap.FRONT_RIGHT_PIVOT,
      Constants.FRONT_RIGHT_P,
      Constants.FRONT_RIGHT_I,
      Constants.FRONT_RIGHT_D,
      Constants.FRONT_RIGHT_KS,
      Constants.FRONT_RIGHT_KV,
      Constants.FRONT_RIGHT_KA,
      Constants.FRONT_RIGHT_DRIVE_P,
      Constants.FRONT_RIGHT_DRIVE_I,
      Constants.FRONT_RIGHT_DRIVE_D,
      Constants.FRONT_RIGHT_DRIVE_KS,
      Constants.FRONT_RIGHT_DRIVE_KV,
      Constants.FRONT_RIGHT_DRIVE_KA,
      RobotMap.FRONT_RIGHT_DIGITAL_INPUT);
  public static final SwerveModule rearLeft = new SwerveModule(
      "RL",
      RobotMap.REAR_LEFT_DRIVE,
      RobotMap.REAR_LEFT_PIVOT,
      Constants.REAR_LEFT_P,
      Constants.REAR_LEFT_I,
      Constants.REAR_LEFT_D,
      Constants.REAR_LEFT_KS,
      Constants.REAR_LEFT_KV,
      Constants.REAR_LEFT_KA,
      Constants.REAR_LEFT_DRIVE_P,
      Constants.REAR_LEFT_DRIVE_I,
      Constants.REAR_LEFT_DRIVE_D,
      Constants.REAR_LEFT_DRIVE_KS,
      Constants.REAR_LEFT_DRIVE_KV,
      Constants.REAR_LEFT_DRIVE_KA,
      RobotMap.REAR_LEFT_DIGITAL_INPUT);
  public static final SwerveModule rearRight = new SwerveModule(
      "RR",
      RobotMap.REAR_RIGHT_DRIVE,
      RobotMap.REAR_RIGHT_PIVOT,
      Constants.REAR_RIGHT_P,
      Constants.REAR_RIGHT_I,
      Constants.REAR_RIGHT_D,
      Constants.REAR_RIGHT_KS,
      Constants.REAR_RIGHT_KV,
      Constants.REAR_RIGHT_KA,
      Constants.REAR_RIGHT_DRIVE_P,
      Constants.REAR_RIGHT_DRIVE_I,
      Constants.REAR_RIGHT_DRIVE_D,
      Constants.REAR_RIGHT_DRIVE_KS,
      Constants.REAR_RIGHT_DRIVE_KV,
      Constants.REAR_RIGHT_DRIVE_KA,
      RobotMap.REAR_RIGHT_DIGITAL_INPUT);

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
    photonCamera = new PhotonWrapper("cameraOne");

    targetPid = new PIDController(Constants.TARGET_P, Constants.TARGET_I, Constants.TARGET_D);
    targetPid.enableContinuousInput(-180.0, 180.0);
    targetPid.setTolerance(1);
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
              fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation())
                  : new ChassisSpeeds(xSpeed, ySpeed, rot));

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

  public Command goToNode(int apriltag, int node) {

    Translation3d nodeTrans = Field.getNodeCoordinatesFieldRelative(apriltag, node);
    Translation2d goal = new Translation2d(
        Field.fieldLayout.getTagPose(apriltag).get().getTranslation().getX() + Field.DIST_FROM_NODE_X_METERS,
        nodeTrans.getY());
    PathPlannerTrajectory trajToGoal = PathPlanner.generatePath(
        new PathConstraints(2, 3),
        PathPoint.fromCurrentHolonomicState(getPose(), getChassisSpeeds()),
        new PathPoint(goal, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180))); // position, heading(direction of
                                                                                      // travel), holonomic rotation
    return followTrajectoryCommand(trajToGoal, false);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isAutoPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          if (isAutoPath) {
            this.poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(),
                trajectory.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommandReversed(
            trajectory,
            this::getPose,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            this::setModuleStates,
            true,
            this));
  }

  public void setPID(double p, double i, double d) {
    targetPid.setPID(p, i, d);
  }

  public void rotateToAngleInPlace(double setAngle) {
    holdAngleWhileDriving(0, 0, setAngle, false);
  }

  public void holdAngleWhileDriving(double x, double y, double setAngle, boolean fod) {
    var rotateOutput = MathUtil.clamp(targetPid.calculate(getYaw(), normalizeAngle(setAngle)), -1, 1)
        * kMaxAngularSpeed;
    this.drive(x, y, rotateOutput, fod);
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
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getAngle(), getSwerveModulePositions());

    Optional<EstimatedRobotPose> estimatedPose = photonCamera.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    if (estimatedPose.isPresent()) {
      EstimatedRobotPose camPose = estimatedPose.get();
      double minimum = 1;
      for(var target : camPose.targetsUsed) {
        double ambiguity = target.getPoseAmbiguity();
        if(ambiguity < minimum) {
          minimum = ambiguity;
        }
      }

      if(minimum < 0.2 && poseEstimator.getEstimatedPosition().getTranslation().minus(getPose().getTranslation()).getNorm() < 1) {
        double stdDev = minimum + 0.05;

        var stdDevMatrix = new Matrix<N3, N1>(new SimpleMatrix(new double[][] {
          { stdDev },
          { stdDev },
          { stdDev }
        }));
      
        poseEstimator.setVisionMeasurementStdDevs(stdDevMatrix);
        poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      }
    }
  }

  public void resetOdometry() {
    poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(), new Pose2d(2, 4, new Rotation2d(Math.PI)));
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

  public boolean resetEncoders() {
    boolean fl = frontLeft.resetEncoder();
    boolean fr = frontRight.resetEncoder();
    boolean rl = rearLeft.resetEncoder();
    boolean rr = rearRight.resetEncoder();
    return fl && fr && rl && rr;
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
    RobotContainer.field.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putNumber("navx yaw", getYaw());
    SmartDashboard.putNumber("navx pitch", RobotContainer.getRobotPitch());
    SmartDashboard.putNumber("nax roll", RobotContainer.getRobotRoll());
    SmartDashboard.putNumber("Pose angle", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("module 1", getSwerveModulePositions()[0].angle.getDegrees());
    SmartDashboard.putNumber("module 2", getSwerveModulePositions()[1].angle.getDegrees());
    SmartDashboard.putNumber("module 3", getSwerveModulePositions()[2].angle.getDegrees());
    SmartDashboard.putNumber("module 4", getSwerveModulePositions()[3].angle.getDegrees());
    SmartDashboard.putBoolean("Calibrated", isCalibrated);
  }

}