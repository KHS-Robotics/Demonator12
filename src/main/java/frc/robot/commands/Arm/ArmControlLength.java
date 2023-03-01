/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

@SuppressWarnings("GrazieInspection")
public class ArmControlLength extends CommandBase {
  double length;

  public ArmControlLength(double length) {
    addRequirements(RobotContainer.arm);
    this.length = length;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if(length < Constants.MIN_LENGTH) {
        length = Constants.MIN_LENGTH;
    }

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.arm.setLength(length);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.arm.getLength() - length) < 0.03;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }
}