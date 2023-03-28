/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RotateToAngle extends CommandBase {
  private Timer timer = new Timer();

  private double angle, error;

  /**
   * Creates a new RotateToAngle.
   */
  public RotateToAngle(double angle) {
    this(angle, 5);
  }

  public RotateToAngle(double angle, double error) {
    this.angle = angle;
    this.error = error;
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetPid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.rotateToAngleInPlace(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var atSetpoint = RobotContainer.swerveDrive.atSetpoint(error);
    if (atSetpoint) {
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }

    return atSetpoint;
  }
}
