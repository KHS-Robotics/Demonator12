package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
  private Timer timer = new Timer();
  private boolean isTimerRunning;

  private boolean reverse;
  private double slopeTolerance = 5.8;
  private final double balanceSpeedMetersPerSecond = 0.3, levelPitch = 5, elapsedTimeToConsiderLevelInSeconds = 1;
  private double yaw, slope, currentPitch, previousPitch;
  private static final double dt = 0.02;

  public AutoBalance(double yaw, boolean reverse) {
    this.addRequirements(RobotContainer.swerveDrive);
    this.yaw = yaw;
    this.reverse = reverse;
  }

  public AutoBalance(double yaw) {
    this(yaw, false);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    currentPitch = RobotContainer.getRobotPitch();
    slope = (currentPitch - previousPitch) / dt;
    SmartDashboard.putNumber("AutoBalanceSlope", slope);

    var isStable = Math.abs(slope) < slopeTolerance;
    var isTilted = Math.abs(currentPitch) > levelPitch;

    SmartDashboard.putBoolean("AutoBalanceIsStable", isStable);
    SmartDashboard.putBoolean("isTilted", isTilted);


    var isAscending = isStable && isTilted;

    SmartDashboard.putBoolean("AutoBalanceIsAscending", isAscending);
    if (isAscending) {
      var xSpeed = (reverse ? -1 : 1) * Math.signum(currentPitch) * balanceSpeedMetersPerSecond;
      RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, 0, yaw, false);
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
    stopTimer();
    RobotContainer.swerveDrive.lock();
  }

  private void stopTimer() {
    timer.stop();
    timer.reset();
  }
}
