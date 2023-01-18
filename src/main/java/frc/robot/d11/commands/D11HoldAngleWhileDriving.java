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

public class D11HoldAngleWhileDriving extends CommandBase {
  private double angle;
  private boolean isFieldOriented;

  /**
   * Creates a new RotateToAngle.
   */
  public D11HoldAngleWhileDriving() {
    addRequirements(D11RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    D11RobotContainer.swerveDrive.resetPid();
    angle = D11RobotContainer.swerveDrive.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpeed = D11RobotContainer.swerveDrive.sensControl(-D11RobotContainer.xboxController.getLeftY()) * D11SwerveDrive.kMaxSpeed;

    var ySpeed = D11RobotContainer.swerveDrive.sensControl(-D11RobotContainer.xboxController.getLeftX()) * D11SwerveDrive.kMaxSpeed;

    isFieldOriented = (!D11RobotContainer.xboxController.getLeftBumper());

    D11RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angle, isFieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}