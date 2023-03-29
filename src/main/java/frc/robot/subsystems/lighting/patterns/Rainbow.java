package frc.robot.subsystems.lighting.patterns;

import frc.robot.Constants;
import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class Rainbow extends LEDPattern {
  public Rainbow() {
    super(40, "Rainbow");
  }

  @Override
  public void setPixels() {
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      LEDStrip.setHSVMirrored((i + this.tick) % Constants.LED_LENGTH, (int) (((double) i / Constants.LED_LENGTH) * 180), 255, 255);
    }
  }
}
