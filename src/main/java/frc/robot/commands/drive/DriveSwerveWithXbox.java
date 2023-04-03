/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;

@SuppressWarnings("GrazieInspection")
public class DriveSwerveWithXbox extends CommandBase {
  private boolean fieldRelative = false;

  public DriveSwerveWithXbox() {
    this.addRequirements(RobotContainer.swerveDrive);
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
    var xSpeed = 0.0;
    if (Math.abs(RobotContainer.driverController.getLeftY()) > 0.05) {
      xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftY()) * SwerveDrive.kMaxSpeedMetersPerSecond;
    }
    SmartDashboard.putNumber("xSpeed", xSpeed);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = 0.0;
    if (Math.abs(RobotContainer.driverController.getLeftX()) > 0.05) {
      ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftX())  * SwerveDrive.kMaxSpeedMetersPerSecond;
    }
    SmartDashboard.putNumber("ySpeed", ySpeed);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.

    // Deadband on new controller
    double rot = 0;
    if (Math.abs(RobotContainer.driverController.getRightX()) > 0.05) {
      rot = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getRightX()) * SwerveDrive.kMaxAngularSpeedRadiansPerSecond;
    }

    fieldRelative = (RobotContainer.driverController.getRightTriggerAxis() < 0.3);
    RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative);
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