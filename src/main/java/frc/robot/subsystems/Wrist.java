package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private final CANSparkMax pivotMotor;
    public Wrist() {
        pivotMotor = new CANSparkMax(RobotMap.WRIST_PIVOT, MotorType.kBrushless);
    }

    public void goToAngle(Rotation2d angle) {
        //do this
    }

    public void goToAbsoluteAngle(Rotation2d absoluteAngle) {
        //do this
    }

    public void setVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    public Rotation2d getAngle() {
        return new Rotation2d();
    }

    public Rotation2d absoluteToRelativeAngle(Rotation2d absoluteRotation) {
        return Rotation2d.fromRadians(RobotContainer.arm.getAngle().getRadians() - getAngle().getRadians());
    }

    public Rotation2d relativeToAbsoluteAngle(Rotation2d relativeAngle) {
        return Rotation2d.fromRadians(getAngle().getRadians() - RobotContainer.arm.getAngle().getRadians());
    }
}
