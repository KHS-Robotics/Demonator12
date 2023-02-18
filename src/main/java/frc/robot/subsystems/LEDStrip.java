package frc.robot.subsystems;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class LEDStrip extends SubsystemBase {
    AddressableLED strip;
    AddressableLEDBuffer buffer;
    int numberSections;

    public LEDStrip(int numberSections) {
        strip = new AddressableLED(RobotMap.LED_PORT);
        strip.setLength(Constants.LED_LENGTH);
        this.numberSections = numberSections;
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

    public void runBlue() {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
            double alternate = (255 / 2) * ((-Math.cos((2 * Math.PI * numberSections * i) / Constants.LED_LENGTH)) + 1);
            buffer.setRGB(i, 0, (int) alternate, 255);
        }
    }

    public void runRed() {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
            double alternate = (255 / 2) * ((-Math.cos((2 * Math.PI * numberSections * i) / Constants.LED_LENGTH)) + 1);
            buffer.setRGB(i, 255, 0, (int) alternate);
        }
    }

    @Override
    public void periodic() {
        if (!DriverStation.isFMSAttached() || DriverStation.getAlliance().equals(Alliance.Invalid)) {
            setYellow();
            return;
        }
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            runRed();
            return;
        }
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            runBlue();
            return;
        }
    }
    
}
