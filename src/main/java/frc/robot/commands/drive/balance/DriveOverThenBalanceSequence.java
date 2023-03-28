package frc.robot.commands.drive.balance;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmHoldSetpoint;
import frc.robot.commands.drive.RotateToAngle;

/**
 * For autonomous
 */
public class DriveOverThenBalanceSequence extends SequentialCommandGroup {
    public DriveOverThenBalanceSequence() {
        super(
          new PrintCommand("Starting approach charge station"),
          new ApproachChargeStation(180, true).finallyDo((inter) -> {
            if (inter) CommandScheduler.getInstance().cancelAll();
          }).withTimeout(2.5),
          new PrintCommand("Now driving forward"),
          new DriveForward(180, true, 1).withTimeout(2.2),
          new WaitCommand(0.4),
          new PrintCommand("Now starting balance seq"),
          new BalanceSequence(180)
          //RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).finallyDo((interrupted) -> new ArmHoldSetpoint()).andThen( new InstantCommand(() -> RobotContainer.wrist.setAngleSetpoint(Rotation2d.fromDegrees(80))))
        );
      }
}
