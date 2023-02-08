/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.drive.SwerveDrive;

@SuppressWarnings("GrazieInspection")
public class GoToNode extends CommandBase {

  int apriltag;
  int node;
  Translation3d nodeTrans;
  Translation2d goal;

  public GoToNode(int apriltag, int node) {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    final double DIST_FROM_NODE_X_METERS = Units.inchesToMeters(35.28);
      nodeTrans = Field.getNodeCoordinatesFieldRelative(apriltag, node);
      goal = new Translation2d(Field.APRILTAGS[apriltag - 1].getX() + DIST_FROM_NODE_X_METERS, nodeTrans.getY());
      PathPlannerTrajectory trajToGoal = PathPlanner.generatePath(
    new PathConstraints(4, 3), 
    new PathPoint(RobotContainer.swerveDrive.getPose().getTranslation(), Rotation2d.fromDegrees(RobotContainer.swerveDrive.getHeading()), Rotation2d.fromDegrees(RobotContainer.swerveDrive.getPose().getRotation().getDegrees())),  // position, heading(direction of travel), holonomic rotation
    new PathPoint(goal, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))); // position, heading(direction of travel), holonomic rotation
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}