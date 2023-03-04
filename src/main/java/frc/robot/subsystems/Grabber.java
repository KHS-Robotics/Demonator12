package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Grabber extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final DoubleSolenoid grabSolenoid;

    public Grabber() {
        intakeMotor = new CANSparkMax(RobotMap.GRABBER_INTAKE, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        
        grabSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.GRABBER_SOLENOID_FORWARD, RobotMap.GRABBER_SOLENOID_REVERSE);
    }

    public void intake(double speed) {
        intakeMotor.setVoltage(12 * speed);
    }

    public void outtake(double speed) {
        intakeMotor.setVoltage(12 * speed);
    }

    public void stopMotor() {
        intakeMotor.disable();
    }

    public void grip() {
        grabSolenoid.set(Value.kForward);
    }

    public void release() {
        grabSolenoid.set(Value.kReverse);
    }
}
