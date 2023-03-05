/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.RotateToAngle;

public class BalanceOnChargeStation extends SequentialCommandGroup {
  public BalanceOnChargeStation(double yaw) {
    this.addCommands(new RotateToAngle(yaw), new ApproachChargeStation(yaw), new AutoBalance(yaw));
  }
}
