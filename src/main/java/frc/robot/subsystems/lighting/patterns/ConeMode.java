package frc.robot.subsystems.lighting.patterns;

import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class ConeMode extends LEDPattern {
  public ConeMode() {
    super(20, "ConeMode");
  }

  @Override
  public void setPixels() {
    this.ticksPerSecond = 20;
    LEDStrip.setAllRGB(255, 255, 0);
    stop();
  }
}
