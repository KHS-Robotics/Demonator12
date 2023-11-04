/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CenterSwerveModules extends CommandBase {
  private boolean force;

  /**
   * Creates a new CenterSwerveModules.
   */
  public CenterSwerveModules(boolean force) {
    this.addRequirements(RobotContainer.swerveDrive);
    this.force = force;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.stop();
    RobotContainer.swerveDrive.isCalibrated = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.swerveDrive.isCalibrated) {
      RobotContainer.swerveDrive.isCalibrated = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stop();
    RobotContainer.swerveDrive.isCalibrated = !interrupted;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.swerveDrive.isCalibrated;
  }
}