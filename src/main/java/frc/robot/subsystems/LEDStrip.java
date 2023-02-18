package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDStrip extends SubsystemBase {
    AddressableLED strip;
    AddressableLEDBuffer buffer;

    public LEDStrip() {
        strip = new AddressableLED(0);
        strip.setLength(Constants.LED_LENGTH);
        
    }

    public void setAllRed() {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
            buffer.setRGB(i, 255, 0, 0);
        }
    }

    public void setAllBlue() {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
            buffer.setRGB(i, 0, 0, 255);
        }
    }

    public void setPurple() {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
            buffer.setRGB(i, 255, 0, 255);
        }
    }

    public void setYellow() {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
            buffer.setRGB(i, 255, 255, 0);
        }
    }

    public void 

    

    @Override
    public void periodic() {
        
    }
    
}
