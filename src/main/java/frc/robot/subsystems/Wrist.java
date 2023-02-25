package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private final CANSparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;

    private final SparkMaxLimitSwitch forwardLimitSwitch;
    private final SparkMaxLimitSwitch reverseLimitSwitch;

    private final PIDController wristPID;
    private Rotation2d angleSetpoint;


    public Wrist() {
        pivotMotor = new CANSparkMax(RobotMap.WRIST_PIVOT, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPositionConversionFactor(2 * Math.PI / Constants.WRIST_GEARING);
        pivotEncoder.setVelocityConversionFactor(2 * Math.PI / Constants.WRIST_GEARING); //encoder in radians
        wristPID = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);
        forwardLimitSwitch = pivotMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        reverseLimitSwitch = pivotMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        forwardLimitSwitch.enableLimitSwitch(false);
        reverseLimitSwitch.enableLimitSwitch(false);
        

        
    }

    public void goToAngle(Rotation2d angle) {
        pivotMotor.setVoltage(wristPID.calculate(pivotEncoder.getPosition(), angle.getRadians()));
        SmartDashboard.putNumber("voltage", wristPID.calculate(pivotEncoder.getPosition(), angle.getRadians()));
    }

    public void goToAbsoluteAngle(Rotation2d absoluteAngle) {
        pivotMotor.setVoltage(wristPID.calculate(pivotEncoder.getPosition(), toRelativeAngle(absoluteAngle).getRadians()));
    }

    public void setVelocity(double angularVelocity) {
        pivotMotor.setVoltage(wristPID.calculate(pivotEncoder.getVelocity(), angularVelocity));
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
        pivotMotor.setVoltage(0);
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
      SmartDashboard.putNumber("wristVelocity", pivotEncoder.getPosition());
    }
}
