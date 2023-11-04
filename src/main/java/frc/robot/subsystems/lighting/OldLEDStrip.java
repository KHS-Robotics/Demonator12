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
import frc.robot.subsystems.drive.SwerveDrive;

public class OldLEDStrip extends SubsystemBase {
  Thread t;
  AddressableLED strip;
  AddressableLEDBuffer buffer;
  int numberSections;
  int counter;
  int ticksPerSecond = 50;
  //int stackercount = 0;
  //Color[] stacker = new Color[Constants.LED_LENGTH];
  public OldLEDStrip() {
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
    strip.setLength(Constants.LED_LENGTH * 2);
    buffer = new AddressableLEDBuffer(Constants.LED_LENGTH * 2);
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored(i, 255, 255, 255);
    }
    strip.setData(buffer);
    strip.start();
    this.numberSections = Constants.LED_LENGTH;
    t.start();
  }

  public void setRGBMirrored(int index, int r, int g, int b) {
    buffer.setRGB(index, r, g, b);
    buffer.setRGB(Constants.LED_LENGTH + index, r, g, b);
  }

  public void setHSVMirrored(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
    buffer.setHSV(Constants.LED_LENGTH + index, h, s, v);
  }

  public void setAllRed() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored(i, 255, 0, 0);
    }
  }

  public void setAllOff() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored(i, 0, 0, 0);
    }
  }

  public void setAllBlue() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored(i, 0, 0, 255);
    }
  }

  public void setAllAllianceColor() {
    if(DriverStation.getAlliance() == Alliance.Red) {
      setAllRed();
    } else {
      setAllBlue();
    }
  }

  public void setPurple() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored(i, 255, 0, 255);
    }
  }

  public void setYellow() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored(i, 255, 255, 0);
    }
  }

  public void runBlue() {
    ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored((i + counter) % Constants.LED_LENGTH, 0, 0, (int) ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1) * 50);
      //double alternate = (255 / 1) * ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1);
      //setRGBMirrored((i + counter) % Constants.LED_LENGTH, 0, (int) alternate, 255);
    }
  }

  
  public void calibratePattern() {
    ticksPerSecond = 10;
        boolean frontLeftCalibrated = !SwerveDrive.frontLeft.isHomed();
        for (int i = 0; i < 3; i++) {
            setRGBMirrored(i, frontLeftCalibrated ? 0 : 255, frontLeftCalibrated ? 255 : 0, 0);
        }
        boolean frontRightCalibrated = !SwerveDrive.frontRight.isHomed();
        for (int i = 4; i < 7; i++) {
            setRGBMirrored(i, frontRightCalibrated ? 0 : 255, frontRightCalibrated ? 255 : 0, 0);
        }
        boolean rearLeftCalibrated = !SwerveDrive.rearLeft.isHomed();
        for (int i = 8; i < 11; i++) {
            setRGBMirrored(i, rearLeftCalibrated ? 0 : 255, rearLeftCalibrated ? 255 : 0, 0);
        }
        boolean rearRightCalibrated = !SwerveDrive.rearRight.isHomed();
        for (int i = 12; i < 15; i++) {
            setRGBMirrored(i, rearRightCalibrated ? 0 : 255, rearRightCalibrated ? 255 : 0, 0);
        }
  }

  /*
  public void blueStack() {
    ticksPerSecond = 10;
    
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      stacker[i % Constants.LED_LENGTH] = Color.kBlue;
    }
  }
  */
  public void runRed() {
    ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGBMirrored((i + counter) % Constants.LED_LENGTH, (int) ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1) * 50, 0, 0);

      //double alternate = (25 / 2) * ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1);
      //setRGBMirrored((i + counter) % Constants.LED_LENGTH, 255, 0, (int) alternate);
    }
  }

  public void runRainbow() {
    ticksPerSecond = 50;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setHSVMirrored((i + counter) % Constants.LED_LENGTH, (int) (((double) i / Constants.LED_LENGTH) * 180), 255, 255);
    }
  }

  public void runSilly() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setHSVMirrored(i, (int) (Math.random() * 180), 255, 255);
    }
  }

  public void update() {
    if (RobotContainer.swerveDrive != null && !RobotContainer.swerveDrive.isCalibrated) {
      calibratePattern();
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