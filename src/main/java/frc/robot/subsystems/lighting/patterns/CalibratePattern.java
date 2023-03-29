package frc.robot.subsystems.lighting.patterns;

import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.lighting.LEDPattern;
import frc.robot.subsystems.lighting.LEDStrip;

public class CalibratePattern extends LEDPattern {
    public CalibratePattern() {
        super(20, "CalibratePattern");
    }

    @Override
    public void setPixels() {
        this.ticksPerSecond = 20;

        boolean frontLeftCalibrated = SwerveDrive.frontLeft.setDetection.get();
        for (int i = 0; i < 3; i++) {
            LEDStrip.setRGBMirrored(i, frontLeftCalibrated ? 0 : 255, frontLeftCalibrated ? 255 : 0, 0);
        }
        LEDStrip.setRGBMirrored(3, 255, 100, 75);
        boolean frontRightCalibrated = SwerveDrive.frontRight.setDetection.get();
        for (int i = 4; i < 7; i++) {
            LEDStrip.setRGBMirrored(i, frontRightCalibrated ? 0 : 255, frontRightCalibrated ? 255 : 0, 0);
        }
        LEDStrip.setRGBMirrored(7, 255, 100, 75);
        boolean rearLeftCalibrated = SwerveDrive.rearLeft.setDetection.get();
        for (int i = 8; i < 11; i++) {
            LEDStrip.setRGBMirrored(i, rearLeftCalibrated ? 0 : 255, rearLeftCalibrated ? 255 : 0, 0);
        }
        LEDStrip.setRGBMirrored(11, 255, 100, 75);
        boolean rearRightCalibrated = SwerveDrive.rearRight.setDetection.get();
        for (int i = 12; i < 15; i++) {
            LEDStrip.setRGBMirrored(i, rearRightCalibrated ? 0 : 255, rearRightCalibrated ? 255 : 0, 0);
        }
        for (int i = 15; i < Constants.LED_LENGTH; i++) {
            LEDStrip.setRGBMirrored(i, 255, 100, 75);
        }
    }
}