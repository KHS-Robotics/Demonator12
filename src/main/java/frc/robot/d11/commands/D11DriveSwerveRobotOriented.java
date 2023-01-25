/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.d11.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.d11.D11RobotContainer;
import frc.robot.d11.subsystems.D11SwerveDrive;

@SuppressWarnings("GrazieInspection")
public class D11DriveSwerveRobotOriented extends CommandBase {
  private boolean fieldRelative = true;

  public D11DriveSwerveRobotOriented() {
    this.addRequirements(D11RobotContainer.swerveDrive);
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
    var xSpeed = D11RobotContainer.swerveDrive.sensControl(-D11RobotContainer.xboxController.getLeftY()) * D11SwerveDrive.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = D11RobotContainer.swerveDrive.sensControl(-D11RobotContainer.xboxController.getLeftX()) * D11SwerveDrive.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.

    //Deadband on new controller
    double rot = 0;
    if (Math.abs(D11RobotContainer.xboxController.getRightX()) > 0.05) {
      rot = D11RobotContainer.swerveDrive.sensControl(-D11RobotContainer.xboxController.getRightX()) * D11SwerveDrive.kMaxAngularSpeed;
    }

    fieldRelative = (D11RobotContainer.xboxController.getRightTriggerAxis() < 0.3);
    D11RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative);
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