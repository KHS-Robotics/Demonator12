package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class LEDStrip extends SubsystemBase {
  Thread t;
  AddressableLED strip;
  AddressableLEDBuffer buffer;
  int numberSections;
  int counter;
  int ticksPerSecond = 50;

  public LEDStrip() {
    t = new Thread(() -> {
      long lastTime = System.nanoTime();
      double delta = 0;

      // very accurate loop, is this bad for performance?
      while (!Thread.interrupted()) {
        double ns = 1000000000 / (double) ticksPerSecond;
        long now = System.nanoTime();
        delta += (now - lastTime) / ns;
        lastTime = now;

        if (delta >= 1) {
          update();
          delta--;
        }
      }
    });
    strip = new AddressableLED(RobotMap.LED_PORT);
    strip.setLength(Constants.LED_LENGTH);
    buffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      buffer.setRGB(i, 255, 255, 255);
    }
    strip.setData(buffer);
    strip.start();
    this.numberSections = Constants.LED_LENGTH;
    t.start();
  }

  public void setAllRed() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      buffer.setRGB(i, 255, 0, 0);
    }
  }

  public void setAllBlue() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      buffer.setRGB(i, 0, 0, 255);
    }
  }

  public void setPurple() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      buffer.setRGB(i, 255, 0, 255);
    }
  }

  public void setYellow() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      buffer.setRGB(i, 255, 255, 0);
    }
  }

  public void runBlue() {
    ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      double alternate = (255 / 2) * ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1);
      buffer.setRGB((i + counter) % Constants.LED_LENGTH, 0, (int) alternate, 255);
    }
  }

  public void runRed() {
    ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      double alternate = (40 / 2) * ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1);
      buffer.setRGB((i + counter) % Constants.LED_LENGTH, 255, (int) alternate, 0);
    }
  }

  public void runRainbow() {
    ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      buffer.setHSV((i + counter) % Constants.LED_LENGTH, (int) (((double) i / Constants.LED_LENGTH) * 180), 255, 255);
    }
  }

  public void update() {
    if (RobotContainer.swerveDrive != null && !RobotContainer.swerveDrive.isCalibrated) {
      runRainbow();
    }
    else if (RobotState.isDisabled() || RobotState.isAutonomous()) {
      if(DriverStation.getAlliance().equals(Alliance.Red)) {
        runRed();
      } else {
        runBlue();
      }
    }
    else if (RobotContainer.operatorBox.coneMode()) {
      setYellow();
    }
    else if (RobotContainer.operatorBox.cubeMode()) {
      setPurple();
    }

    strip.setData(buffer);

    counter++;
  }

  @Override
  public void periodic() {
    // this runs off thread so it doesn't stutter
  }
}
