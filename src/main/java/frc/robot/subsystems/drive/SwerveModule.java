/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Swerve Module
 */
@SuppressWarnings("GrazieInspection")
public class SwerveModule extends SubsystemBase {
  public final String name;

  private boolean isFlipped;

  private final CANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;

  private final CANCoder pivotEncoder;
  private final CANSparkMax pivotMotor;
  private final SparkMaxPIDController drivePID;
  private final SimpleMotorFeedforward driveFeedForward;
  // private final SimpleMotorFeedforward pivotFeedForward;

  private final PIDController pivotPID;

  private double offsetAngle;

  /**
   * Constructs a Swerve Module.
   *
   * @param name              the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP            P value of Pivot PID
   * @param pivotI            I value of Pivot PID
   * @param pivotD            D value of Pivot PID
   * @param driveP            P value of Drive PID
   * @param driveI            I value of Drive PID
   * @param driveD            D value of Drive PID
   * @param pivotEncoderId    port number for the pivot CANCoder
   * @param reversed          true if drive motor is reversed
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI,
      double pivotD, double driveP, double driveI, double driveD,
      double drivekS, double drivekV, double drivekA, int pivotEncoderId, boolean reversed, double offsetAngle) {

    this.name = name;

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    pivotMotor = new CANSparkMax(pivotMotorChannel, MotorType.kBrushless);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    pivotMotor.setSmartCurrentLimit(30);
    driveMotor.setSmartCurrentLimit(65);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);

    pivotEncoder = new CANCoder(pivotEncoderId);
    pivotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    driveEncoder = driveMotor.getEncoder();
    driveMotor.setInverted(false);
    driveEncoder.setVelocityConversionFactor(Constants.DRIVE_VEL_ENCODER); // 4" diameter wheel (0.0508 meter radius), , in meters/minute so divide by 60 to get meters/seconds
                                                                           // 8.33:1 -> 2*pi*0.0508 / (8.33 * 60)
    driveEncoder.setPositionConversionFactor(Constants.DRIVE_POS_ENCODER); // 4" diameter wheel (0.0508 meter radius)
                                                                           // 8.33:1 -> 2*pi*0.0508 / 8.33

    drivePID = driveMotor.getPIDController();
    drivePID.setP(driveP);
    drivePID.setI(driveI);
    drivePID.setD(driveD);

    driveFeedForward = new SimpleMotorFeedforward(drivekS, drivekV, drivekA);

    pivotPID = new PIDController(pivotP, pivotI, pivotD);

    pivotPID.enableContinuousInput(-180, 180);
    pivotPID.setTolerance(1);
    this.offsetAngle = offsetAngle;
  }

  /**
   * Constructs a Swerve Module.
   *
   * @param name              the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP            P value of Pivot PID
   * @param pivotI            I value of Pivot PID
   * @param pivotD            D value of Pivot PID
   * @param digitalInputPort  port number for the digital input, used to calibrate
   *                          pivots
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI,
      double pivotD, double driveP, double driveI, double driveD,
      double drivekS, double drivekV, double drivekA, int pivotEncoderId, double offsetAngle) {
    this(name, driveMotorChannel, pivotMotorChannel, pivotP, pivotI, pivotD, driveP, driveI,
        driveD, drivekS, drivekV, drivekA, pivotEncoderId, false, offsetAngle);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber(name + "-Drive", getState().speedMetersPerSecond);
    //SmartDashboard.putNumber(name + "-Pivot", getAngle());
    SmartDashboard.putBoolean("SwerveModule-" + name + "-Homed", isHomed());
    
    // var state = getState();
    // SmartDashboard.putNumber("SwerveModule-" + name + "-Translation", state.speedMetersPerSecond);
    // SmartDashboard.putNumber("SwerveModule-" + name + "-Rotation", state.angle.getDegrees());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Sets the PID values for the pivot module.
   *
   * @param p the p value for the pivot module
   * @param i the i value for the pivot module
   * @param d the d value for the pivot module
   */
  public void setPid(double p, double i, double d) {
    pivotPID.setPID(p, i, d);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state           desired state with the speed and angle
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(SwerveModuleState state, boolean useShortestPath) {
    drivePID.setReference(state.speedMetersPerSecond * (isFlipped && useShortestPath ? -1 : 1),
        CANSparkMax.ControlType.kVoltage, 1,
        driveFeedForward.calculate(state.speedMetersPerSecond * (isFlipped && useShortestPath ? -1 : 1)));
    pivotMotor.set(MathUtil.clamp(pivotPID.calculate(getAngle(),
        useShortestPath ? calculateShortestPath(state.angle.getDegrees()) : state.angle.getDegrees()), -1, 1));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state desired state with the speed and angle
   */
  public void setDesiredState(SwerveModuleState state) {
    setDesiredState(state, true);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param speed           the desired speed in meters/second
   * @param angle           the desired angle in degrees from [-180, 180]
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(double speed, double angle, boolean useShortestPath) {
    setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)), useShortestPath);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param speed the desired speed in meters/second
   * @param angle the desired angle in degrees from [-180, 180]
   */
  public void setDesiredState(double speed, double angle) {
    setDesiredState(speed, angle, true);
  }

  /**
   * Gets the angle of the pivot module.
   *
   * @return the angle of the pivot module ranging from [-180,180]
   */
  public double getAngle() {
    double angle = pivotEncoder.getAbsolutePosition() + offsetAngle;

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

  /**
   * Stops the module.
   */
  public void stop() {
    driveMotor.set(0);
    pivotMotor.set(0);
    pivotPID.reset();
  }

  /**
   * Gets if the module is at the home position using the talon tach.
   * @return true if module is homed, false otherwise
   */
  public boolean isHomed() {
    // sensor "inverted"
    return Math.abs(this.getAngle()) <= 0.5;
  }

  /**
   * Calculates the shortest path the pivot module should take, it
   * might be the given <code>targetAngle</code>. Flips the drive motor
   * if there is a shorter path.
   *
   * @param targetAngle the desired angle of the module
   * @return the shortest path to the target angle, flips the
   *         drive motor if there is a shorter path
   */
  private double calculateShortestPath(double targetAngle) {
    var currentAngle = this.getAngle();
    var dAngle = Math.abs(targetAngle - currentAngle);

    isFlipped = dAngle > 90 && dAngle < 270;

    if (isFlipped) {
      if (targetAngle > 0 || targetAngle == 0 && currentAngle < 0) {
        targetAngle -= 180;
      } else if (targetAngle < 0 || targetAngle == 0 && currentAngle > 0) {
        targetAngle += 180;
      }
    }

    return targetAngle;
  }
}
