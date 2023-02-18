/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.drive.SwerveDrive;

@SuppressWarnings("GrazieInspection")
public class MoveLength extends CommandBase {
    double length;
    double initTime;
    TrapezoidProfile profile;

    public MoveLength(double length) {
        addRequirements(RobotContainer.arm);
        this.length = length;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10),
                                                new TrapezoidProfile.State(length, 0),
                                                new TrapezoidProfile.State(RobotContainer.arm.getLength(), RobotContainer.arm.getLengthV()));
        initTime = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        TrapezoidProfile.State state = profile.calculate(Timer.getFPGATimestamp() - initTime);
        RobotContainer.arm.setExtendVoltage(RobotContainer.arm.calcExtend(state.velocity));
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(RobotContainer.arm.getLength() - length) < 0.04;
        
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }
}