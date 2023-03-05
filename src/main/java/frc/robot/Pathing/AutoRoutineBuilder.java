// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathing;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.balance.BalanceSequence;

/**
 * Used to build autonomous routines.
 */
public class AutoRoutineBuilder {
    private static final HashMap<String, Command> AutonomousEventMap = new HashMap<>();

    /** Gets the event map for PathPlanner's FollowPathWithEvents. */
    private static HashMap<String, Command> getAutonomousEventMap() {
        if (AutonomousEventMap.isEmpty()) {
            AutonomousEventMap.put("Intake", new PrintCommand("placeholder for intake command"));
            AutonomousEventMap.put("PlaceHigh", new PrintCommand("placeholder for place high"));
            AutonomousEventMap.put("PlaceMid", new PrintCommand("placeholder for place mid"));
            AutonomousEventMap.put("PlaceHybrid", new PrintCommand("placeholder for place hybrid"));
            AutonomousEventMap.put("BalanceFacingAway", new BalanceSequence(0));
            AutonomousEventMap.put("BalanceFacingAwayReverse", new BalanceSequence(0, true));
            AutonomousEventMap.put("BalanceFacingDriver", new BalanceSequence(180));
            AutonomousEventMap.put("BalanceFacingDriverReverse", new BalanceSequence(180, true));
            AutonomousEventMap.put("ScoreAngle", RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH));
        }

        return AutonomousEventMap;
    }

    /***
     * The X-Coordinate PID Controller for the {@link CustomSwerveControllerCommand}
     */
    public static final PIDController SwerveXPIDController = new PIDController(0.8, 0.001, 0.8);
    /***
     * The Y-Coordinate PID Controller for the {@link CustomSwerveControllerCommand}
     */
    public static final PIDController SwerveYPIDController = new PIDController(0.8, 0.001, 0.8);
    /***
     * The Theta (rotation) PID Controller for the {@link CustomSwerveControllerCommand}
     */
    public static final ProfiledPIDController SwerveThetaPIDController = new ProfiledPIDController(3.5, 0.001, 0.0,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    public static PathPlannerTrajectory Place2CableProtectorEngage = PathPlanner.loadPath("Place 2 Cable Protector Engage",
            new PathConstraints(2, 3));
    public static PathPlannerTrajectory Place2CableProtector = PathPlanner.loadPath("Place 2 Cable Protector",
            new PathConstraints(2, 3));
    public static PathPlannerTrajectory Place2LoadingStationEngage = PathPlanner.loadPath("Place 2 Loading Station Engage",
            new PathConstraints(2, 3));
    public static PathPlannerTrajectory Place2LoadingStation = PathPlanner.loadPath("Place 2 Loading Station",
            new PathConstraints(2, 3));
    public static PathPlannerTrajectory Place3CableProtector = PathPlanner.loadPath("Place 3 Cable Protector",
            new PathConstraints(2, 3));
    public static PathPlannerTrajectory Place3LoadingStation = PathPlanner.loadPath("Place 3 Loading Station",
            new PathConstraints(2, 3));
    public static PathPlannerTrajectory PlaceEngageLeave = PathPlanner.loadPath("Place and Engage+", new PathConstraints(2, 3));
    public static PathPlannerTrajectory PlaceEngage = PathPlanner.loadPath("Place and Engage", new PathConstraints(2, 3));

    public static Command getAutonomousCommand(PathPlannerTrajectory trajectory) {
        FollowPathWithEvents command = new FollowPathWithEvents(generateSwerveCommand(trajectory), trajectory.getMarkers(), getAutonomousEventMap());
        return command;
    }

    private static SwerveControllerCommand generateSwerveCommand(Trajectory trajectory) {
        return new SwerveControllerCommand(
                trajectory,
                RobotContainer.swerveDrive::getPose,
                RobotContainer.swerveDrive.kinematics,
                SwerveXPIDController,
                SwerveYPIDController,
                SwerveThetaPIDController,
                RobotContainer.swerveDrive::setModuleStates,
                RobotContainer.swerveDrive
        );
    }
}
