package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WristHoldSetpoint extends CommandBase {
  private Rotation2d setpoint;

  public WristHoldSetpoint() {
    this.addRequirements(RobotContainer.wrist);
  }

  @Override
  public void initialize() {
    RobotContainer.wrist.wristSetpoint = new TrapezoidProfile.State(
        RobotContainer.wrist.getRelativeAngle().getRadians(), RobotContainer.wrist.getVelocity());
    RobotContainer.wrist.wristPID.reset();
  }

  @Override
  public void execute() {
    // System.out.println("WRIST HOLD!");
    this.setpoint = RobotContainer.wrist.getAngleSetpoint();
    RobotContainer.wrist.goToAbsoluteAngle(setpoint);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
