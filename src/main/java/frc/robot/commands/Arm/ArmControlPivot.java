/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

@SuppressWarnings("GrazieInspection")
public class ArmControlPivot extends CommandBase {
  double angle;

  public ArmControlPivot(double angle) {
    addRequirements(RobotContainer.arm);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.arm.setAngle(angle);
    RobotContainer.arm.setLength(RobotContainer.arm.armLengthSetpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.arm.getAngle().getRadians() - angle) < 0.03) || !RobotContainer.arm.isLegalHeight(RobotContainer.arm.getTranslation());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.armPivotSetpointRadians = angle;
  }
}