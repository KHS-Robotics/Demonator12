package frc.robot.commands.drive.balance;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmHoldSetpoint;
import frc.robot.commands.drive.RotateToAngle;

/**
 * For autonomous
 */
public class DriveOverThenBalanceSequence extends ParallelDeadlineGroup {
    public DriveOverThenBalanceSequence() {
        super(
          new SequentialCommandGroup(new ApproachChargeStation(180, true),
           new DriveForward(180, true, 1).withTimeout(2.7),
            new BalanceSequence(180))
          /*RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).finallyDo((interrupted) -> new ArmHoldSetpoint()).andThen(
        new InstantCommand(() -> RobotContainer.wrist.setAngleSetpoint(Rotation2d.fromDegrees(80))))*/
        );
      }
}
