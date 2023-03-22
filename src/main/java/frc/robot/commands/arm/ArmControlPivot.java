/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

@SuppressWarnings("GrazieInspection")
public class ArmControlPivot extends CommandBase {
  Supplier<Double> angle;

  public ArmControlPivot(Supplier<Double> angle) {
    addRequirements(RobotContainer.arm);
    this.angle = angle;
  }

  public ArmControlPivot(double angle) {
    addRequirements(RobotContainer.arm);
    this.angle = () -> angle;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotContainer.arm.resetPivotPID();
    RobotContainer.arm.resetExtendPID();
    
    RobotContainer.arm.armLengthSetpoint = RobotContainer.arm.getLength();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    SmartDashboard.putNumber("ArmControlPivotCmdSetpoint", angle.get());
    RobotContainer.arm.setAngle(angle.get());
    RobotContainer.arm.setLength(RobotContainer.arm.armLengthSetpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.arm.getAngle().getRadians() - angle.get()) < Math.toRadians(3));
        //|| !RobotContainer.arm.isLegalHeight(RobotContainer.arm.getTranslation());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.armPivotSetpointRadians = angle.get();
  }
}