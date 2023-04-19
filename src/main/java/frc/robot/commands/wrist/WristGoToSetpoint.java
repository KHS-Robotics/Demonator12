package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WristGoToSetpoint extends CommandBase {
  private Rotation2d setpoint;

  public WristGoToSetpoint(Rotation2d angle) {
    this.addRequirements(RobotContainer.wrist);
    this.setpoint = angle;
  }

  @Override
  public void initialize() {
    RobotContainer.wrist.setAngleSetpoint(setpoint);
    RobotContainer.wrist.wristSetpoint = new TrapezoidProfile.State(
        RobotContainer.wrist.getRelativeAngle().getRadians(), RobotContainer.wrist.getVelocity());
    RobotContainer.wrist.wristPID.reset();
  }

  @Override
  public void execute() {

    RobotContainer.wrist.goToAbsoluteAngle(setpoint);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.wrist.getAbsoluteAngle().getRadians() - setpoint.getRadians()) < 0.06;
  }

  @Override
  public void end(boolean interrupted) {
    // new WristHoldSetpoint().schedule();
  }
}
