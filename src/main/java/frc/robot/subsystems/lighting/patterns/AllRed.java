package frc.robot.subsystems.lighting.patterns;

import frc.robot.Constants;
import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class AllRed extends LEDPattern {
  public AllRed() {
    super(20, "AllRed");
  }

  @Override
  public void setPixels() {
    this.ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      LEDStrip.setRGBMirrored(i, 255, 0, 0);
    }
    stop();
  }
}
