package frc.robot.subsystems.lighting.patterns;

import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class CubeMode extends LEDPattern {
  public CubeMode() {
    super(20, "CubeMode");
  }

  @Override
  public void setPixels() {
    this.ticksPerSecond = 20;
    LEDStrip.setAllRGB(255, 0, 255);
    stop();
  }
}
