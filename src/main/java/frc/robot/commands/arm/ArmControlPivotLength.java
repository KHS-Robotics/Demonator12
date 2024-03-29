/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

@SuppressWarnings("GrazieInspection")
public class ArmControlPivotLength extends CommandBase {
  double angle;
  double length;

  public ArmControlPivotLength(double angle, double length) {
    addRequirements(RobotContainer.arm);
    this.angle = angle;
    this.length = length;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotContainer.arm.pivotSetpoint = new TrapezoidProfile.State(RobotContainer.arm.getAngle().getRadians(),
        RobotContainer.arm.getAngleV());
    RobotContainer.arm.lengthSetpoint = new TrapezoidProfile.State(RobotContainer.arm.getLength(),
        RobotContainer.arm.getLengthV());
      

    RobotContainer.arm.resetPivotPID();
    RobotContainer.arm.resetExtendPID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotContainer.arm.setAngle(angle);
    RobotContainer.arm.setLength(length);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.arm.getAngle().getRadians() - angle) < Math.toRadians(3)) && (Math.abs(RobotContainer.arm.getLength() - length) < 0.03);
        //|| !RobotContainer.arm.isLegalHeight(RobotContainer.arm.getTranslation());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.armLengthSetpoint = length;
    RobotContainer.arm.armPivotSetpointRadians = angle;
  }
}