package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
    private Timer timer = new Timer();

    private boolean reverse;
    private double slopeTolerance = 0.33;
    private final double balanceSpeedMetersPerSecond = 0.3, levelPitch = 2, elapsedTimeToConsiderLevelInSeconds = 0.5;
    private double yaw, slope, currentPitch, previousPitch;
    private static final double dt = 0.02;

    public AutoBalance(double yaw, boolean reverse) {
        this.addRequirements(RobotContainer.swerveDrive);
        this.yaw = yaw;
        this.reverse = reverse;
        this.slopeTolerance = (reverse ? 1 : -1) * slopeTolerance;
    }

    public AutoBalance(double yaw) {
        this(yaw, false);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        currentPitch = RobotContainer.navx.getPitch();
        slope = (currentPitch - previousPitch) / dt;

        var isAscending = slope < slopeTolerance;
        if (isAscending) {
            var xSpeed = (reverse ? -1 : 1) * Math.signum(currentPitch) * balanceSpeedMetersPerSecond;
            RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, 0, yaw, false);
            stopTimer();
        } else {
            RobotContainer.swerveDrive.stop();
            timer.start();
        }

        previousPitch = currentPitch;
    }

    @Override
    public boolean isFinished() {
        var hasTimeElapsed = timer.advanceIfElapsed(elapsedTimeToConsiderLevelInSeconds);
        var isSlopeWithinTolerance = slope < slopeTolerance;
        var isPitchWithinTolerance = Math.abs(currentPitch) < levelPitch;

        return hasTimeElapsed && isSlopeWithinTolerance && isPitchWithinTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        stopTimer();
    }

    private void stopTimer() {
        timer.stop();
        timer.reset();
    }
}
