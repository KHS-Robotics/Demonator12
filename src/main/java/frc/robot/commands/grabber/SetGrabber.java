/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class SetGrabber extends SequentialCommandGroup {
  public SetGrabber(boolean release) {
    this.addCommands(
      release ? new InstantCommand(() -> RobotContainer.grabber.grip())
            : new InstantCommand(() -> RobotContainer.grabber.release()),
        new WaitCommand(0.5), new InstantCommand(() -> RobotContainer.grabber.turnOffGrabSolenoid()));
  }
}
