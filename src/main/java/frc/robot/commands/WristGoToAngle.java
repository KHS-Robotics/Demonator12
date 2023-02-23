package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class WristGoToAngle extends CommandBase {
    Rotation2d angle;

    public WristGoToAngle(Rotation2d angle) {
        this.addRequirements(RobotContainer.wrist);
        this.angle = angle;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        RobotContainer.wrist.goToAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(RobotContainer.wrist.getRelativeAngle().getRadians() - angle.getRadians()) < 0.05;

    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.wrist.stop();
    }
    
}
