/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

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
    var xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftY()) * SwerveDrive.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftX()) * SwerveDrive.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.

    //Deadband on new controller
    double rot = 0;
    if (Math.abs(RobotContainer.driverController.getRightX()) > 0.05) {
      rot = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getRightX()) * SwerveDrive.kMaxAngularSpeed;
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