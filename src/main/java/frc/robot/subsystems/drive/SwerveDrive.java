/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;


import org.ejml.dense.row.mult.SubmatrixOps_FDRM;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  private Matrix<N3, N1> stdDevMatrix;

  public PhotonWrapper photonCamera;

  public boolean isCalibrated = false;

  public static final SwerveModule frontLeft = new SwerveModule(
      "FL",
      RobotMap.FRONT_LEFT_DRIVE,
      RobotMap.FRONT_LEFT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.PIVOT_KS,
      Constants.PIVOT_KV,
      Constants.PIVOT_KA,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.FRONT_LEFT_DIGITAL_INPUT);
  public static final SwerveModule frontRight = new SwerveModule(
      "FR",
      RobotMap.FRONT_RIGHT_DRIVE,
      RobotMap.FRONT_RIGHT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.PIVOT_KS,
      Constants.PIVOT_KV,
      Constants.PIVOT_KA,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.FRONT_RIGHT_DIGITAL_INPUT);
  public static final SwerveModule rearLeft = new SwerveModule(
      "RL",
      RobotMap.REAR_LEFT_DRIVE,
      RobotMap.REAR_LEFT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.PIVOT_KS,
      Constants.PIVOT_KV,
      Constants.PIVOT_KA,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.REAR_LEFT_DIGITAL_INPUT);
  public static final SwerveModule rearRight = new SwerveModule(
      "RR",
      RobotMap.REAR_RIGHT_DRIVE,
      RobotMap.REAR_RIGHT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.PIVOT_KS,
      Constants.PIVOT_KV,
      Constants.PIVOT_KA,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
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

    stdDevMatrix = new Matrix<N3, N1>(new SimpleMatrix(new double[][] {
      { 0.3 },
      { 0.3 },
      { 45 }
    }));
    poseEstimator.setVisionMeasurementStdDevs(stdDevMatrix);
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
    /*SmartDashboard.putNumber("GoToNodeTag", apriltag);
    SmartDashboard.putNumber("GoToNodeNode", node);
    Rotation2d heading;


    Translation3d nodeTrans = Field.getNodeCoordinatesFieldRelative(apriltag, node);

    SmartDashboard.putNumber("GoToNodeTransX", nodeTrans.getX());
    SmartDashboard.putNumber("GoToNodeTransY", nodeTrans.getY());
    ChassisSpeeds currentSpeeds = getChassisSpeeds();

    double linearVel =
        Math.sqrt(
            (currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond)
                + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));


    Translation2d goal = new Translation2d(
        Field.fieldLayout.getTagPose(apriltag).get().getTranslation().getX() + Field.DIST_FROM_NODE_X_METERS,
        
    nodeTrans.getY());
    if(getPose().getY() > goal.getY()) {
      heading = Rotation2d.fromDegrees(-90);
     } else {
      heading = Rotation2d.fromDegrees(90);
     }

    PathPoint initialPoint = new PathPoint(
      getPose().getTranslation(), heading, getPose().getRotation(), linearVel);
    PathPlannerTrajectory trajToGoal = PathPlanner.generatePath(
        new PathConstraints(1, 1.5),
        //PathPoint.fromCurrentHolonomicState(getPose(), getChassisSpeeds()),
        initialPoint,
        new PathPoint(goal, Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), 0)); // position, heading(direction of
                                                                                      // travel), holonomic rotation
    //return followTrajectoryCommand(trajToGoal, false);
    return RobotContainer.swerveAutoBuilder.followPath(trajToGoal);*/
    return new PrintCommand("gotonode");
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(), pose);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isAutoPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          if (isAutoPath) {
            this.poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(),
                PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), DriverStation.getAlliance()).poseMeters);
          }
        }),
        new PPSwerveControllerCommandReversed(
            trajectory,
            this::getPose,
            new PIDController(0, 0, 0.1),
            new PIDController(0, 0, 0.1),
            new PIDController(0, 0, 0),
            this::setModuleStates,
            true,
            this));
  }

  public void setPID(double p, double i, double d) {
    targetPid.setPID(p, i, d);
  }

  public void rotateToAngleInPlace(double setAngle) {
    holdAngleWhileDriving(0, 0, Rotation2d.fromDegrees(setAngle), false);
  }

  public void holdAngleWhileDriving(double x, double y, Rotation2d setAngle, boolean fieldOriented) {
    var rotateOutput = MathUtil.clamp(targetPid.calculate(getYaw(), normalizeAngle(setAngle.getDegrees())), -1, 1) * kMaxAngularSpeed;
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
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getAngle(), getSwerveModulePositions());

    PhotonPipelineResult result = photonCamera.getResult();
    if(result.hasTargets()) {
      var target = result.getBestTarget();
      double ambiguity = target.getPoseAmbiguity();
      int fiducialID = target.getFiducialId();
      
      if (photonCamera.fieldLayout.getTagPose(fiducialID).isPresent()) {
        Pose2d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), photonCamera.fieldLayout.getTagPose(fiducialID).get(), Constants.CAMERA_1_POS.inverse()).toPose2d();
        if(ambiguity < 0.1 && robotPose.getTranslation().getDistance(photonCamera.fieldLayout.getTagPose(fiducialID).get().getTranslation().toTranslation2d()) < 2.5/*&& poseEstimator.getEstimatedPosition().getTranslation().getDistance(getPose().getTranslation()) < 1*/) {
          poseEstimator.addVisionMeasurement(robotPose, result.getTimestampSeconds());
        }
      }
    }
  }
  

  public void resetOdometry() {
    RobotContainer.navx.reset();
    offset = Math.toDegrees(Math.PI);
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