package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
    private final double tolerance = 0.33, levelAngle = 2;
    private final double balanceSpeedMetersPerSecond = 0.3;
    private double yaw, slope, currentPitch, previousPitch;
    private static final double dt = 0.02;

    public AutoBalance(double yaw) {
        this.addRequirements(RobotContainer.swerveDrive);
        this.yaw = yaw;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        currentPitch = RobotContainer.navx.getPitch();
        slope = (currentPitch - previousPitch) / dt;
        if (slope < tolerance) {
            var xSpeed = Math.signum(currentPitch) * balanceSpeedMetersPerSecond;
            RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, 0, yaw, false);
        } else {
            RobotContainer.swerveDrive.stop();
        }

        previousPitch = currentPitch;
    }

    @Override
    public boolean isFinished() {
        return slope < tolerance && Math.abs(currentPitch) < levelAngle;
    }

    @Override
    public void end(boolean interrupted) {}
}
