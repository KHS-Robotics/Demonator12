package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveForward extends CommandBase {

    double yaw;
    double speed;
    boolean reverse;

  public DriveForward(double yaw, boolean reverse, double speed) {
    this.addRequirements(RobotContainer.swerveDrive);
    this.yaw = yaw;
    this.reverse = reverse;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
      var xSpeed = (reverse ? -1 : 1) * speed;
      RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, 0, yaw, false);
  }

  @Override
  public boolean isFinished() {

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}
