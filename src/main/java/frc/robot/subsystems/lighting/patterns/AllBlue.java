package frc.robot.subsystems.lighting.patterns;

import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class AllBlue extends LEDPattern {
  public AllBlue() {
    super(20, "AllBlue");
  }

  @Override
  public void setPixels() {
    this.ticksPerSecond = 20;
    LEDStrip.setAllRGB(0, 0, 255);
    LEDStrip.update();
    stop();
  }
}
