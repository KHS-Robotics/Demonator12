package frc.robot.subsystems.lighting.patterns;

import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class AllOff extends LEDPattern {
  public AllOff() {
    super(20, "AllOff");
  }

  @Override
  public void setPixels() {
    LEDStrip.setAllRGB(0, 0, 0);
    stop();
  }
}
