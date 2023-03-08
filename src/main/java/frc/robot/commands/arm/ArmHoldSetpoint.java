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

public class ArmHoldSetpoint extends CommandBase {
    public ArmHoldSetpoint() {
        addRequirements(RobotContainer.arm);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        RobotContainer.arm.pivotSetpoint = new TrapezoidProfile.State(RobotContainer.arm.getAngle().getRadians(), RobotContainer.arm.getAngleV());
        RobotContainer.arm.lengthSetpoint = new TrapezoidProfile.State(RobotContainer.arm.getLength(), RobotContainer.arm.getLengthV());
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        RobotContainer.arm.setAngle(RobotContainer.arm.armPivotSetpointRadians);
        RobotContainer.arm.setLength(RobotContainer.arm.armLengthSetpoint);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {}
}