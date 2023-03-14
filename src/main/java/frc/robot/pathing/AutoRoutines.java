// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathing;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.balance.BalanceSequence;

/**
 * Autos
 */
public class AutoRoutines {
    /***
     * The X-Coordinate PID Controller for the {@link CustomSwerveControllerCommand}
     */
    public static final PIDController SwerveXPIDController = new PIDController(0.8, 0.001, 0.8);
    /***
     * The Y-Coordinate PID Controller for the {@link CustomSwerveControllerCommand}
     */
    public static final PIDController SwerveYPIDController = new PIDController(0.8, 0.001, 0.8);
    /***
     * The Theta (rotation) PID Controller for the
     * {@link CustomSwerveControllerCommand}
     */
    public static final ProfiledPIDController SwerveThetaPIDController = new ProfiledPIDController(3.5, 0.001, 0.0,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI));

//     public static List<PathPlannerTrajectory> Place2CableProtectorEngage = PathPlanner.loadPathGroup(
//             "Place 2 Cable Protector Engage",
//             new PathConstraints(2, 3));
//     public static List<PathPlannerTrajectory> Place2CableProtector = PathPlanner.loadPathGroup("Place 2 Cable Protector",
//             new PathConstraints(2, 3));
//     public static List<PathPlannerTrajectory> Place2LoadingStationEngage = PathPlanner.loadPathGroup(
//             "Place 2 Loading Station Engage",
//             new PathConstraints(2, 3));
//     public static List<PathPlannerTrajectory> Place2LoadingStation = PathPlanner.loadPathGroup("Place 2 Loading Station",
//             new PathConstraints(2, 3));

    public static List<PathPlannerTrajectory> PlaceEngageLeave = PathPlanner.loadPathGroup("Place + Mobility + Engage",
            new PathConstraints(2, 3));
    public static List<PathPlannerTrajectory> PlaceEngage = PathPlanner.loadPathGroup("Place + Engage",
            new PathConstraints(2, 3));
    public static List<PathPlannerTrajectory> PlaceMobilityCableProtector = PathPlanner.loadPathGroup("Place + Mobility (Cable Protector)",
            new PathConstraints(2, 3));
    public static List<PathPlannerTrajectory> TestPath = PathPlanner.loadPathGroup("Testing",
            new PathConstraints(0.5, 0.5));

    public static Command getPlaceHighCenterThenAutoEngage() {
        return new SequentialCommandGroup(
            new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS)),
            RobotContainer.arm.goToPivotLength(Math.toRadians(35), Constants.MIN_LENGTH),
            new BalanceSequence(180, true)
        );
    }
}
