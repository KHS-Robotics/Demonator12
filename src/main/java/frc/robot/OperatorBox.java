package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Field.GridPosition;

public class OperatorBox extends Joystick {

  public OperatorBox(int port) {
    super(port);
  }

  public GridPosition getGrid() {
    if (this.getRawButton(8)) { return GridPosition.LEFT; }
    if (this.getRawButton(9)) { return GridPosition.RIGHT; }
    return GridPosition.MID;
  }

  public int getHeight() {
    if (this.getRawButton(6)) { return 2; }
    if (this.getRawButton(7)) { return 0; }
    return 1;
  }

  public boolean leftNode() {
    return this.getRawButton(10 /* LEFT NODE BUTTON ID */);
  }

  public boolean cubeNode() {
    return this.getRawButton(11 /* CUBE NODE BUTTON ID */);
  }

  public boolean rightNode() {
    return this.getRawButton(12 /* RIGHT NODE BUTTON ID */);
  }

  public boolean abort() {
    return this.getRawButton(2 /* ABORT BUTTON ID */);
  }

  public boolean highConeKnockedOver() {
    return this.getRawButton(3 /* ZERO ARM LENGTH BUTTON ID */);
  }

  // THIS SHOULD NOT BE BOUND
  // public boolean zeroArmPivot() {
  //   return this.getRawButton(0 /* ZERO ARM PIVOT BUTTON ID */);
  // }
  

  public boolean midConeKnockedOver() {
    return this.getRawButton(4 /* ZERO WRIST PIVOT BUTTON ID */);
  }

  public boolean cubeMode() {
    return (this.getRawButton(5));
  }

  public boolean coneMode() {
    return !(this.getRawButton(5));
  }

}
