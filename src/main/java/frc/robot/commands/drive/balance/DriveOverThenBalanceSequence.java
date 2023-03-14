package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.RotateToAngle;

public class DriveOverThenBalanceSequence extends SequentialCommandGroup {
    public DriveOverThenBalanceSequence() {
        this.addCommands(
          new RotateToAngle(0),
          new ApproachChargeStation(0),
          new DriveForward(0, false, 0.7).withTimeout(3.5),
          new BalanceSequence(180)
        );
      }
}
