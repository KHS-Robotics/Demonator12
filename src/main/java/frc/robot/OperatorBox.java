package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OperatorBox extends Joystick {

    public OperatorBox(int port) {
        super(port);
    }

    public boolean holdWrist() {
        return this.getRawButton(0);
      }
    
}
