package frc.robot.subsystems.lighting;

import java.util.HashMap;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
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
      double alternate = (255 / 1) * ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1);
      setRGBMirrored((i + counter) % Constants.LED_LENGTH, 0, (int) alternate, 255);
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
      double alternate = (25 / 2) * ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1);
      setRGBMirrored((i + counter) % Constants.LED_LENGTH, 255, 0, (int) alternate);
    }
  }

  public void runRainbow() {
    ticksPerSecond = 50;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setHSVMirrored((i + counter) % Constants.LED_LENGTH, (int) (((double) i / Constants.LED_LENGTH) * 180), 255, 255);
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

  public void readMessageMorse(String message) {
    HashMap<Character, String> morseMap = new HashMap<>();
    morseMap.put('a', ".-");
    morseMap.put('b', "-...");
    morseMap.put('c', "-.-.");
    morseMap.put('d', "-..");
    morseMap.put('e', ".");
    morseMap.put('f', "..-.");
    morseMap.put('g', "--.");
    morseMap.put('h', "....");
    morseMap.put('i', "..");
    morseMap.put('j', ".---");
    morseMap.put('k', "-.-");
    morseMap.put('l', ".-..");
    morseMap.put('m', "--");
    morseMap.put('n', "-.");
    morseMap.put('o', "---");
    morseMap.put('p', ".--.");
    morseMap.put('q', "--.-");
    morseMap.put('r', ".-.");
    morseMap.put('s', "...");
    morseMap.put('t', "-");
    morseMap.put('u', "..-");
    morseMap.put('v', "...-");
    morseMap.put('w', ".--");
    morseMap.put('x', "-..-");
    morseMap.put('y', "-.--");
    morseMap.put('z', "--..");
    SequentialCommandGroup morseSequentialCommandGroup = new SequentialCommandGroup();
    message = message.toLowerCase();
    for (int i = 0; i < message.length(); i++) {
      char c = message.charAt(i);
      String morseCode = morseMap.get(c);
      if (morseCode != null) {
        for (int j = 0; i < morseCode.length(); i++) {
          if(morseCode.charAt(i) == '-') {
            morseSequentialCommandGroup.addCommands(new InstantCommand(() -> setAllAllianceColor()).andThen(new WaitCommand(3 / ticksPerSecond)).andThen(new InstantCommand(()-> setAllOff())).andThen(new WaitCommand(1 / ticksPerSecond)));
          } else if(morseCode.charAt(i) == '.') {
            morseSequentialCommandGroup.addCommands(new InstantCommand(() -> setAllAllianceColor()).andThen(new WaitCommand(1 / ticksPerSecond)).andThen(new InstantCommand(()-> setAllOff())).andThen(new WaitCommand(1 / ticksPerSecond)));
          }
        }
        morseSequentialCommandGroup.addCommands(new WaitCommand(2 / ticksPerSecond));
      } else {
        morseSequentialCommandGroup.addCommands(new WaitCommand(4 / ticksPerSecond));
      }
    }
    CommandScheduler.getInstance().schedule(morseSequentialCommandGroup);
  }


}