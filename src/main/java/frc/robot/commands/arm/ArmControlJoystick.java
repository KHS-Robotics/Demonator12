/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

@SuppressWarnings("GrazieInspection")
public class ArmControlJoystick extends CommandBase {
  public ArmControlJoystick() {
    addRequirements(RobotContainer.arm);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    var isLegalExtension = RobotContainer.arm.isLegalExtension(RobotContainer.arm.getTranslation());
    var isLegalHeight = RobotContainer.arm.isLegalHeight(RobotContainer.arm.getTranslation());

    RobotContainer.arm.setLengthV(isLegalExtension && isLegalHeight ? RobotContainer.operatorStick.getExtendSpeed() : MathUtil.clamp(RobotContainer.operatorStick.getExtendSpeed(), -1, 0));
    RobotContainer.arm.setAngleV(isLegalExtension ? (isLegalHeight ? RobotContainer.operatorStick.getPitchSpeed() : MathUtil.clamp(RobotContainer.operatorStick.getPitchSpeed(), -1, 0.25)) : MathUtil.clamp(RobotContainer.operatorStick.getPitchSpeed(), 0, 1));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.operatorStick.getX()) <= 0.025 && Math.abs(RobotContainer.operatorStick.getY()) <= 0.025;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.armLengthSetpoint = RobotContainer.arm.getLength();
    RobotContainer.arm.armPivotSetpointRadians = RobotContainer.arm.getAngle().getRadians();
    SmartDashboard.putNumber("ArmLengthSetpoint", RobotContainer.arm.getLength());
    SmartDashboard.putNumber("ArmPivotSetpoint", RobotContainer.arm.getAngle().getRadians());

  }
}