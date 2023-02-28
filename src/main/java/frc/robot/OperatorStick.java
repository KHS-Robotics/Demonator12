package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OperatorStick extends Joystick {

    public OperatorStick(int port) {
        super(port);
    }

    //right side buttons
    public boolean highPos() {
        return this.getRawButton(13);
    }

    public boolean midPos() {
        return this.getRawButton(12);
    }

    public boolean lowPos() {
        return this.getRawButton(11);
    }

    public boolean home() {
        return this.getRawButton(14);
    }

    public boolean stow() {
        return this.getRawButton(15);
    }

    public boolean scoreAngle() {
        return this.getRawButton(16);
    }

    //left side buttons
    public boolean wristStepUp() {
        return this.getRawButton(5);
    }

    public boolean wristUpOverride() {
        return this.getRawButton(6);
    }

    public boolean wristFlat() {
        return this.getRawButton(7);
    }

    public boolean wristStepDown() {
        return this.getRawButton(10);
    }

    public boolean wristDownOverride() {
        return this.getRawButton(9);
    }

    public boolean wristStraightDown() {
        return this.getRawButton(8);
    }

    //stuff on the stick
    public boolean openClaw() {
        return this.getRawButton(2);
    }

    public boolean closeClaw() {
        return this.getRawButton(1);
    }

    public double getPitchSpeed() {
        return this.getRawAxis(1);
    }

    public double getExtendSpeed() {
        return this.getRawAxis(0);
    }




    
}
