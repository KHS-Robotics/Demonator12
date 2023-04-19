package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
  private Timer timer = new Timer();
  private Timer backUpTimer = new Timer();
  private boolean isTimerRunning;

  private double slopeTolerance = 8.5;
  private final double balanceSpeedMetersPerSecond = 0.225, levelPitch = 5, elapsedTimeToConsiderLevelInSeconds = 1;
  private double yaw, slope, currentPitch, previousPitch;
  private static final double dt = 0.02;

  public AutoBalance(double yaw) {
    this.addRequirements(RobotContainer.swerveDrive);
    this.yaw = yaw;
  }

  @Override
  public void initialize() {
    backUpTimer.start();
  }

  @Override
  public void execute() {
    currentPitch = RobotContainer.getRobotPitch();
    SmartDashboard.putNumber("Pitch", currentPitch);
    slope = (currentPitch - previousPitch) / dt;
    SmartDashboard.putNumber("AutoBalanceSlope", slope);

    var isStable = Math.abs(slope) < slopeTolerance;
    var isTilted = Math.abs(currentPitch) > levelPitch;


    var isAscending = isStable && isTilted;

    if (isAscending) {
      var xSpeed = Math.signum(currentPitch) * balanceSpeedMetersPerSecond;
      RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, 0, yaw, false);
      backUpTimer.reset();
    } else if(!backUpTimer.hasElapsed(0.1)) {
      var xSpeed = Math.signum(currentPitch) * balanceSpeedMetersPerSecond;
      RobotContainer.swerveDrive.holdAngleWhileDriving(- xSpeed / 2.0, 0, yaw, false);
    } else {
      RobotContainer.swerveDrive.lock();
    }

    previousPitch = currentPitch;
  }

  @Override
  public boolean isFinished() {
    var isSlopeWithinTolerance = Math.abs(slope) < slopeTolerance;
    var isPitchWithinTolerance = Math.abs(currentPitch) < levelPitch;

    var isRobotPitchWithinTolerance = isSlopeWithinTolerance && isPitchWithinTolerance;

    if (!isTimerRunning && isRobotPitchWithinTolerance) {
      timer.start();
      isTimerRunning = true;
    }

    if (isTimerRunning && !isRobotPitchWithinTolerance) {
      stopTimer();
      isTimerRunning = false;
    }

    var hasTimeElapsed = isTimerRunning && timer.advanceIfElapsed(elapsedTimeToConsiderLevelInSeconds);

    return hasTimeElapsed && isRobotPitchWithinTolerance;
  }

  @Override
  public void end(boolean interrupted) {
    backUpTimer.stop();
    backUpTimer.reset();
    stopTimer();
    RobotContainer.swerveDrive.lock();
  }

  private void stopTimer() {
    timer.stop();
    timer.reset();
  }

}
