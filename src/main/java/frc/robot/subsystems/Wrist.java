package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private final CANSparkMax pivotMotor;
  // private final SparkMaxPIDController wristPidCtrl;
  private final SparkMaxAnalogSensor pivotEncoder;
  private static final double MIN_VOLTAGE = 0.312, MAX_VOLTAGE = 3.25, DELTA_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE;
  private static final double WRIST_CONVERSION_FACTOR = -Math.toRadians(200) / DELTA_VOLTAGE;

  private final SparkMaxLimitSwitch forwardLimitSwitch;
  private final SparkMaxLimitSwitch reverseLimitSwitch;

  public final PIDController wristPID;

  // wrist acts as an "arm" in code
  private final ArmFeedforward wristFeedForward;
  private final Constraints wristConstraints = new TrapezoidProfile.Constraints(1.5, 5);
  public TrapezoidProfile.State wristSetpoint = new TrapezoidProfile.State();
  private static final double kDt = 0.02;
  private static final double OFFSET = 1.98;

  private Rotation2d angleSetpoint = new Rotation2d(); 

  public Wrist() {
    pivotMotor = new CANSparkMax(RobotMap.WRIST_PIVOT, MotorType.kBrushless);

    pivotEncoder = pivotMotor.getAnalog(Mode.kAbsolute);
    pivotEncoder.setPositionConversionFactor(1);
    pivotEncoder.setVelocityConversionFactor(1); // encoder in radians

    /*
     * wristPidCtrl = pivotMotor.getPIDController();
     * wristPidCtrl.setP(Constants.WRIST_P, 0);
     * wristPidCtrl.setI(Constants.WRIST_I, 0);
     * wristPidCtrl.setD(Constants.WRIST_D, 0);
     * wristPidCtrl.setFF(0, 0);
     */
    wristPID = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);

    forwardLimitSwitch = pivotMotor.getForwardLimitSwitch(Type.kNormallyClosed);
    forwardLimitSwitch.enableLimitSwitch(true);

    reverseLimitSwitch = pivotMotor.getReverseLimitSwitch(Type.kNormallyClosed);
    reverseLimitSwitch.enableLimitSwitch(true);

    wristFeedForward = new ArmFeedforward(Constants.WRIST_KS, Constants.WRIST_KG, Constants.WRIST_KV,
        Constants.WRIST_KA);
  }

  public void goToAngle(Rotation2d angle) {
    // wristPidCtrl.setReference(angle.getRadians(),
    // CANSparkMax.ControlType.kPosition);
  }

  public void goToAbsoluteAngle(Rotation2d absoluteAngle) {
    TrapezoidProfile profile = new TrapezoidProfile(wristConstraints,
        new TrapezoidProfile.State(toRelativeAngle(absoluteAngle).getRadians(), 0), wristSetpoint);
    wristSetpoint = profile.calculate(kDt);
    SmartDashboard.putNumber("WristSetpointToRelativeGoal", toRelativeAngle(absoluteAngle).getRadians());
    SmartDashboard.putNumber("wristSetpoint.velocity", wristSetpoint.velocity);
    SmartDashboard.putNumber("wristSetpoint.position", wristSetpoint.position);

    var output = wristFeedForward.calculate(getAbsoluteAngle().getRadians(), wristSetpoint.velocity) + wristPID.calculate(getRelativeAngle().getRadians(), wristSetpoint.position);
    pivotMotor.setVoltage(output);
  }

  public double getVoltage() {
    return pivotEncoder.getVoltage();
  }

  public double getAngle() {
    return (pivotEncoder.getVoltage() * WRIST_CONVERSION_FACTOR) + OFFSET;
  }

  public Rotation2d getRelativeAngle() {
    return new Rotation2d(getAngle());
  }

  public double getVelocity() {
    return (pivotEncoder.getVelocity() * WRIST_CONVERSION_FACTOR);
  }

  public Rotation2d getAbsoluteAngle() {
    return getRelativeAngle().plus(RobotContainer.arm.getAngle());
  }

  public Rotation2d toAbsoluteAngle(Rotation2d relativeAngle) {
    return relativeAngle.plus(RobotContainer.arm.getAngle());
  }

  public Rotation2d toRelativeAngle(Rotation2d absoluteAngle) {
    return absoluteAngle.minus(RobotContainer.arm.getAngle());
  }

  public void stop() {
    pivotMotor.stopMotor();
  }

  public Rotation2d getAngleSetpoint() {
    return angleSetpoint;
  }

  public void setAngleSetpoint(Rotation2d angleSetpoint) {
    this.angleSetpoint = angleSetpoint;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("WristRelativeAngle", getAngle());
    SmartDashboard.putNumber("WristAbsoluteAngle", getAbsoluteAngle().getRadians());
    SmartDashboard.putNumber("WristVelocity", getVelocity());
  }

  public boolean getTopTalonTach() {
    return forwardLimitSwitch.isPressed();
  }

  public boolean getBottomTalonTach() {
    return reverseLimitSwitch.isPressed();
  }
}
