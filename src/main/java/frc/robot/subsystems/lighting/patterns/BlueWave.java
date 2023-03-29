package frc.robot.subsystems.lighting.patterns;

import frc.robot.Constants;
import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class BlueWave extends LEDPattern {
  public BlueWave() {
    super(20, "BlueWave");
  }

  @Override
  public void setPixels() {
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      double alternate = (255 / 1) * ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1);
      LEDStrip.setRGBMirrored((i + this.tick) % Constants.LED_LENGTH, 0, (int) alternate, 255);
    }
  }
}
