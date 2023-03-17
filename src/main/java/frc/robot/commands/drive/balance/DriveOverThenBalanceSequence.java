package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmHoldSetpoint;
import frc.robot.commands.drive.RotateToAngle;

public class DriveOverThenBalanceSequence extends SequentialCommandGroup {
    public DriveOverThenBalanceSequence() {
        this.addCommands(
          (RobotContainer.arm.goToPivotLength(Math.toRadians(15), Constants.MIN_LENGTH).finallyDo((interrupted) -> new ArmHoldSetpoint().schedule())).alongWith(
          new RotateToAngle(180),
          new ApproachChargeStation(180, true),
          new DriveForward(0, true, 1).withTimeout(3),
          new BalanceSequence(180))
        );
      }
}
