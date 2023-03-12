package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;

public class HoldAngleWithXbox extends CommandBase {
    private boolean fieldRelative = false;
    private Rotation2d angleSetpoint;

  public HoldAngleWithXbox() {
    this.addRequirements(RobotContainer.swerveDrive);
    this.angleSetpoint = RobotContainer.swerveDrive.getAngle();
  }

  public HoldAngleWithXbox(Rotation2d angleSetpoint) {
    this.addRequirements(RobotContainer.swerveDrive);
    this.angleSetpoint = angleSetpoint;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftY()) * SwerveDrive.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftX()) * SwerveDrive.kMaxSpeed;
    fieldRelative = (RobotContainer.driverController.getRightTriggerAxis() < 0.3);
    RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angleSetpoint, fieldRelative);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
