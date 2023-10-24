package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

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
          new DriveForward(180, true, 1).withTimeout(2.3),
          new InstantCommand(() -> RobotContainer.swerveDrive.stop()),
          new WaitCommand(1),
          new PrintCommand("Now starting balance seq"),
          new BalanceSequence(180)
          //RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).finallyDo((interrupted) -> new ArmHoldSetpoint()).andThen( new InstantCommand(() -> RobotContainer.wrist.setAngleSetpoint(Rotation2d.fromDegrees(80))))
        );
      }
}
