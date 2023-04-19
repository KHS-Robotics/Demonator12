package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Grabber extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final SparkMaxLimitSwitch intakeSensor;
  private final Solenoid grab, release;
  public boolean waiting;

  public Grabber() {
    intakeMotor = new CANSparkMax(RobotMap.GRABBER_INTAKE, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeSensor = intakeMotor.getForwardLimitSwitch(Type.kNormallyClosed); //THIS IS TEMPORARY WHILE THE SENSOR ISNT MOUNTED PROPER
    intakeSensor.enableLimitSwitch(false);




































    intakeMotor.setInverted(true);

    grab = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.GRABBER_SOLENOID_FORWARD);
    grab.setPulseDuration(0.5);
    release = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.GRABBER_SOLENOID_REVERSE);
    release.setPulseDuration(0.5);
        
    turnOff();
  }

  public void set(double speed) {
      intakeMotor.setVoltage(-12 * speed);
  }

  public void stopMotor() {
    intakeMotor.disable();
  }

  public void grip() {
    grab.set(true);
    release.set(false);
  }

  public void release() {
    release.set(true);
    grab.set(false);
  }

  public void turnOff() {
    release.set(false);
    grab.set(false);
  }

  public void waitForCone() {
    waiting = true;
  }

  public void stopWaitingForCone() {
    waiting = false;
  }

  public boolean getSensor() {
    return !intakeSensor.isPressed();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("waiting", waiting);
    SmartDashboard.putBoolean("isPressed", getSensor());
  }
}
