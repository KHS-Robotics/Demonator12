package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WaitUntilUp extends CommandBase {

  public WaitUntilUp() {
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.arm.getAngle().getRadians() > -0.2;
  }

}
