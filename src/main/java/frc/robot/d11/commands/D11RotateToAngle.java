package frc.robot.d11.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.d11.D11RobotContainer;

public class D11RotateToAngle extends CommandBase {
  double angle, error;

  /**
   * Creates a new RotateToAngle.
   */
  public D11RotateToAngle(double angle) {
    this(angle, 1);
  }

  public D11RotateToAngle(double angle, double error) {
    this.angle = angle;
    this.error = error;
    addRequirements(D11RobotContainer.swerveDrive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    D11RobotContainer.swerveDrive.resetPid();
    //RobotContainer.swerveDrive.rotateToTargetInPlace();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    D11RobotContainer.swerveDrive.rotateToAngleInPlace(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return D11RobotContainer.swerveDrive.atSetpoint(error);
  }
}