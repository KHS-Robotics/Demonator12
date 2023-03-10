package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class WristDeltaSetpoint extends InstantCommand {
  private Rotation2d delta;

  public WristDeltaSetpoint(Rotation2d delta) {
    this.delta = delta;
  }

  @Override
  public void initialize() {
    RobotContainer.wrist.setAngleSetpoint(RobotContainer.wrist.getAngleSetpoint().plus(delta));
  }
}
