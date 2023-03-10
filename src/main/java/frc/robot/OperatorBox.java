package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Field.GridPosition;

public class OperatorBox extends Joystick {

  public OperatorBox(int port) {
    super(port);
  }

  public GridPosition getGrid() {
    double switchVal = this.getRawAxis(0 /* SET TO HORIZONTAL SWITCH AXIS */);
    if (switchVal < -0.5) {
      return GridPosition.LEFT;
    } else if (switchVal > 0.5) {
      return GridPosition.MID;
    } else {
      return GridPosition.RIGHT;
    }
  }

  public int getHeight() {
    double switchVal = this.getRawAxis(0 /* SET TO VERTICAL SWITCH AXIS */);
    if (switchVal < -0.5) {
      return 0;
    } else if (switchVal > 0.5) {
      return 2;
    } else {
      return 1;
    }
  }

  public boolean leftNode() {
    return this.getRawButton(0 /* LEFT NODE BUTTON ID */);
  }

  public boolean cubeNode() {
    return this.getRawButton(0 /* CUBE NODE BUTTON ID */);
  }

  public boolean rightNode() {
    return this.getRawButton(0 /* RIGHT NODE BUTTON ID */);
  }

  public boolean abort() {
    return this.getRawButton(0 /* ABORT BUTTON ID */);
  }

  public boolean zeroArmLength() {
    return this.getRawButton(0 /* ZERO ARM LENGTH BUTTON ID */);
  }

  public boolean zeroArmPivot() {
    return this.getRawButton(0 /* ZERO ARM PIVOT BUTTON ID */);
  }

  public boolean zeroWristPivot() {
    return this.getRawButton(0 /* ZERO WRIST PIVOT BUTTON ID */);
  }

  public boolean cubeMode() {
    return getRawAxis(0 /* MODE SWITCH AXIS */) <= 0;
  }

  public boolean coneMode() {
    return getRawAxis(0 /* MODE SWITCH AXIS */) > 0;
  }

}
