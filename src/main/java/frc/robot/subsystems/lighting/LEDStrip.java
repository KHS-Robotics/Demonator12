package frc.robot.subsystems.lighting;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.lighting.patterns.AllBlue;
import frc.robot.subsystems.lighting.patterns.AllRed;
import frc.robot.subsystems.lighting.patterns.BlueWave;
import frc.robot.subsystems.lighting.patterns.CalibratePattern;
import frc.robot.subsystems.lighting.patterns.ConeMode;
import frc.robot.subsystems.lighting.patterns.CubeMode;
import frc.robot.subsystems.lighting.patterns.DisplayMorse;
import frc.robot.subsystems.lighting.patterns.Rainbow;
import frc.robot.subsystems.lighting.patterns.RedWave;

public class LEDStrip extends SubsystemBase {
  public static LEDPattern active;
  Thread t;
  public static AddressableLED strip;
  public static AddressableLEDBuffer buffer;

  public LEDStrip() {
    strip = new AddressableLED(RobotMap.LED_PORT);
    strip.setLength(Constants.LED_LENGTH * 2);
    buffer = new AddressableLEDBuffer(Constants.LED_LENGTH * 2);
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored(i, 255, 255, 255);
    }
    strip.setData(buffer);
    strip.start();
  }

  public static void setAllRGB(int r, int g, int b) {
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored(i, r, g, b);
    }
  }

  public static void setRGBMirrored(int index, int r, int g, int b) {
    buffer.setRGB(index, r, g, b);
    buffer.setRGB(Constants.LED_LENGTH + index, r, g, b);
  }

  public static void setHSVMirrored(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
    buffer.setHSV(Constants.LED_LENGTH + index, h, s, v);
  }

  public static boolean isRunning() {
    if (active == null) {
      return false;
    }
    return active.isRunning();
  }

  public void setAllAllianceColor() {
    if(DriverStation.getAlliance() == Alliance.Red) {
      new AllRed();
    } else {
      new AllBlue();
    }
  }

  // LEDPattern calls this, don't worry about it when making a new pattern
  public static void update() {
    strip.setData(buffer);
  }

  // should be logic determining which pattern to run, and NOTHING ELSE (LEDs still run off thread so they don't stutter)
  @Override
  public void periodic() {
    if (RobotContainer.swerveDrive != null && !RobotContainer.swerveDrive.isCalibrated) {
      active = new CalibratePattern();
      //if (active == null || !active.isRunning())
      //  active = new DisplayMorse("4342");
    }
    else if (RobotState.isDisabled() || RobotState.isAutonomous()) {
      if(DriverStation.getAlliance().equals(Alliance.Red)) {
        active = new RedWave();
      } else {
        active = new BlueWave();
      }
    }
    else if (RobotContainer.operatorBox.coneMode()) {
      active = new ConeMode();
    }
    else if (RobotContainer.operatorBox.cubeMode()) {
      active = new CubeMode();
    }
    active.run();
  }
}
