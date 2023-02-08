/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.drive.SwerveDrive;

@SuppressWarnings("GrazieInspection")
public class ArmControlSetpoint extends CommandBase {
  Translation3d target;

  public ArmControlSetpoint(Translation3d target) {
    addRequirements(RobotContainer.arm);
    this.target = target;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.arm.setLength(RobotContainer.arm.lengthToPoint(target));
    RobotContainer.arm.setAngle(RobotContainer.arm.rotToPoint(target));
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