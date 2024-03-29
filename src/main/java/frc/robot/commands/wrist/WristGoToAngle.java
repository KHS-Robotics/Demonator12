package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WristGoToAngle extends CommandBase {
  private double tolerance = 0.05;
  private Rotation2d desiredWristAng, desiredArmAng;
  private Rotation2d relativeSetpoint;

  public WristGoToAngle(Rotation2d desiredWristAng, Rotation2d desiredArmAng) {
    this.addRequirements(RobotContainer.wrist);
    this.desiredWristAng = desiredWristAng;
    this.desiredArmAng = desiredArmAng;
    
    
  }

  @Override
  public void initialize() {
    RobotContainer.wrist.wristPID.reset();

    relativeSetpoint = desiredWristAng.minus(desiredArmAng);
    
    RobotContainer.wrist.setRelativeSetpoint(relativeSetpoint);
    RobotContainer.wrist.setAngleSetpoint(desiredWristAng);

    RobotContainer.wrist.wristSetpoint = new TrapezoidProfile.State(
        RobotContainer.wrist.getRelativeAngle().getRadians(), RobotContainer.wrist.getVelocity());
    RobotContainer.wrist.wristPID.reset();
  }

  @Override
  public void execute() {
    RobotContainer.wrist.goToAngle(relativeSetpoint);
  }

  @Override
  public boolean isFinished() {
    var current = RobotContainer.wrist.getRelativeAngle().getRadians();
    return Math.abs(relativeSetpoint.getRadians() - current) < tolerance;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
