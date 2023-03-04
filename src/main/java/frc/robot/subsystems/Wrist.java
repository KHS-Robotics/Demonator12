package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private final CANSparkMax pivotMotor;
    private final SparkMaxPIDController wristPidCtrl;
    private final RelativeEncoder pivotEncoder;

    private final SparkMaxLimitSwitch forwardLimitSwitch;
    private final SparkMaxLimitSwitch reverseLimitSwitch;

    private Rotation2d angleSetpoint;

    public Wrist() {
        pivotMotor = new CANSparkMax(RobotMap.WRIST_PIVOT, MotorType.kBrushless);

        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPositionConversionFactor(2 * Math.PI / Constants.WRIST_GEARING);
        pivotEncoder.setVelocityConversionFactor(2 * Math.PI / Constants.WRIST_GEARING); // encoder in radians

        wristPidCtrl = pivotMotor.getPIDController();
        wristPidCtrl.setP(Constants.WRIST_P);
        wristPidCtrl.setI(Constants.WRIST_I);
        wristPidCtrl.setD(Constants.WRIST_D);
        
        forwardLimitSwitch = pivotMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        forwardLimitSwitch.enableLimitSwitch(false);
        
        reverseLimitSwitch = pivotMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        reverseLimitSwitch.enableLimitSwitch(false);
    }

    public void goToAngle(Rotation2d angle) {
        wristPidCtrl.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    public void goToAbsoluteAngle(Rotation2d absoluteAngle) {
        wristPidCtrl.setReference(toRelativeAngle(absoluteAngle).getRadians(), CANSparkMax.ControlType.kPosition);
    }

    public Rotation2d getRelativeAngle() {
        return new Rotation2d(pivotEncoder.getPosition());
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

    public void zeroWrist() {
        pivotEncoder.setPosition(0);
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
      SmartDashboard.putNumber("wristPos", pivotEncoder.getPosition());
    }
}
