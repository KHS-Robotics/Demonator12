/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class SetGrabber extends SequentialCommandGroup {
  public SetGrabber(boolean grip) {
    this.addCommands(
      grip ? new InstantCommand(() -> RobotContainer.grabber.grip()) :
                new InstantCommand(() -> RobotContainer.grabber.release())
    );
  }
}
