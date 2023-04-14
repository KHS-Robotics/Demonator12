/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

@SuppressWarnings("GrazieInspection")
public class ArmControlLength extends CommandBase {
  Supplier<Double> length;

  public ArmControlLength(Supplier<Double> length) {
    addRequirements(RobotContainer.arm);
    this.length = length;
  }

  public ArmControlLength(double length) {
    addRequirements(RobotContainer.arm);
    this.length = () -> length;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotContainer.arm.resetPivotPID();
    RobotContainer.arm.resetExtendPID();

    if (length.get() < Constants.MIN_LENGTH) {
      length = () -> Constants.MIN_LENGTH ;
    }

    
    RobotContainer.arm.pivotSetpoint = new TrapezoidProfile.State(RobotContainer.arm.getAngle().getRadians(),
        RobotContainer.arm.getAngleV());
    RobotContainer.arm.lengthSetpoint = new TrapezoidProfile.State(RobotContainer.arm.getLength(),
        RobotContainer.arm.getLengthV());

    RobotContainer.arm.armPivotSetpointRadians = RobotContainer.arm.getAngle().getRadians();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.arm.setLength(length.get());
    RobotContainer.arm.setAngle(RobotContainer.arm.armPivotSetpointRadians);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
      return (Math.abs(RobotContainer.arm.getLength() - length.get()) < 0.0625);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.armLengthSetpoint = length.get();
  }
}