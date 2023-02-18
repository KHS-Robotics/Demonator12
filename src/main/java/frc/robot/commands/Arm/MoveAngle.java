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
public class MoveAngle extends CommandBase {
    double angle;
    double initTime;
    TrapezoidProfile profile;

    public MoveAngle(double angle) {
        addRequirements(RobotContainer.arm);
        this.angle = angle;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10),
                                                new TrapezoidProfile.State(angle, 0),
                                                new TrapezoidProfile.State(RobotContainer.arm.getAngle().getRadians(), RobotContainer.arm.getAngleV()));
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
        return Math.abs(RobotContainer.arm.getAngle().getRadians() - angle) < 0.05 / RobotContainer.arm.getLength();
        
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }
}