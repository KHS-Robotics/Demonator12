package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Grabber extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final SparkMaxLimitSwitch intakeSensor;
  private final DoubleSolenoid grabSolenoid;
  public boolean waiting;

  public Grabber() {
    intakeMotor = new CANSparkMax(RobotMap.GRABBER_INTAKE, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeSensor = intakeMotor.getForwardLimitSwitch(Type.kNormallyClosed);
    intakeSensor.enableLimitSwitch(true);

    grabSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.GRABBER_SOLENOID_FORWARD,
        RobotMap.GRABBER_SOLENOID_REVERSE);
    turnOffGrabSolenoid();
  }

  public void set(double speed) {
    intakeMotor.setVoltage(-12 * speed);
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

  public void turnOffGrabSolenoid() {
    grabSolenoid.set(Value.kOff);
  }

  public void waitForCone() {
    waiting = true;
  }

  public void stopWaitingtForCone() {
    waiting = false;
  }

  public boolean getSensor() {
    return intakeSensor.isPressed();
  }
}
