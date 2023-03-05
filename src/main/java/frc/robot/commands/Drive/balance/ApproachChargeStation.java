/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ApproachChargeStation extends CommandBase {
    private static final double TargetPitchToEndCommand = 10;
    private static final double ApporachSpeedMetersPerSecond = 0.5;
    private double yaw;

    public ApproachChargeStation(double yaw) {
        addRequirements(RobotContainer.swerveDrive);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.swerveDrive.resetPid();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.swerveDrive.holdAngleWhileDriving(ApporachSpeedMetersPerSecond, 0, yaw, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // TODO: need to negate pitch?? depends on how it is mounted
        return RobotContainer.navx.getPitch() > TargetPitchToEndCommand;
    }
}
