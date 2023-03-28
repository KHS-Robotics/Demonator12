/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ApproachChargeStation extends CommandBase {
  private Timer timer = new Timer();
  private static final double AbsoluteTargetPitchToEndCommand = 10;
  private static final double ApporachSpeedMetersPerSecond = 1.5;
  private double yaw;
  private boolean reverse;

  public ApproachChargeStation(double yaw, boolean reverse) {
    addRequirements(RobotContainer.swerveDrive);
    this.yaw = yaw;
    this.reverse = reverse;
  }

  public ApproachChargeStation(double yaw) {
    this(yaw, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetPid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.holdAngleWhileDriving((reverse ? -1 : 1) * ApporachSpeedMetersPerSecond, 0, yaw,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var done = Math.abs(RobotContainer.getRobotPitch()) > AbsoluteTargetPitchToEndCommand;
    if (done) {
      timer.start();
    }
    else {
      timer.stop();
      timer.reset();
    }

    return done;// && timer.hasElapsed(0.2);
  }
}
