package frc.robot.subsystems.lighting.patterns;

import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class AllRed extends LEDPattern {
  public AllRed() {
    super(20, "AllRed");
  }

  @Override
  public void setPixels() {
    this.ticksPerSecond = 20;
    LEDStrip.setAllRGB(255, 0, 0);
    stop();
  }
}
