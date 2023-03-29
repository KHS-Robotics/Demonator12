package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.wrist.WristGoToSetpoint;
import frc.robot.commands.wrist.WristHoldSetpoint;

public class ArmGoToSetpointFurther extends SequentialCommandGroup {
  private Rotation2d wristAngle;
  private Translation3d target;
  private double rotToPoint, lengthToPoint;

  public ArmGoToSetpointFurther(Rotation2d wristAngle, Translation3d target) {
    this.wristAngle = wristAngle;
    this.target = target;

    this.addCommands(
        new WristGoToSetpoint(wristAngle),
        (new ArmControlPivot(rotToPoint).andThen(new ArmControlLength(lengthToPoint)))
            .deadlineWith(new WristHoldSetpoint()));
  }
}
