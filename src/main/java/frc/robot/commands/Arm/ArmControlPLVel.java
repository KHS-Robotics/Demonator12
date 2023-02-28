/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.drive.SwerveDrive;

@SuppressWarnings("GrazieInspection")
public class ArmControlPLVel extends CommandBase {
  double angleV;
  double lengthV;

  public ArmControlPLVel(double angleV, double lengthV) {
    addRequirements(RobotContainer.arm);
    this.angleV = angleV;
    this.lengthV = lengthV;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.arm.setLengthV(lengthV);
    RobotContainer.arm.setAngleV(angleV);
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