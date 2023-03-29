package frc.robot.subsystems.lighting.patterns;

import frc.robot.Constants;
import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class RedWave extends LEDPattern {
  public RedWave() {
    super(20, "BlueWave");
  }

  @Override
  public void setPixels() {
    ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      double alternate = (25 / 2) * ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1);
      LEDStrip.setRGBMirrored((i + LEDStrip.counter) % Constants.LED_LENGTH, 255, 0, (int) alternate);
    }
  }
}
