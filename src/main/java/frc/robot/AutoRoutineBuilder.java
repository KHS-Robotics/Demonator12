// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drive.SwerveDrive;

/**
 * Used to build autonomous routines.
 *
 * @see AutonomousRoutine
 * @see AutoRoutineBuilder#build()
 * @see frc.robot.subsystems.SwerveDrive
 * @see frc.robot.subsystems.SwerveDrive#getPose()
 * @see frc.robot.subsystems.SwerveDrive#kinematics
 * @see frc.robot.subsystems.SwerveDrive#setModuleStates(edu.wpi.first.math.kinematics.SwerveModuleState[])
 * @see frc.robot.subsystems.SwerveDrive#resetNavx(Pose2d)
 */
public class AutoRoutineBuilder {

    static HashMap<String, Command> eventMap = new HashMap<>();

    public static void setEventMap() {
        eventMap.put("intake", new PrintCommand("placeholder for intake command"));
        eventMap.put("placehigh", new PrintCommand("placeholder for place high"));
        eventMap.put("placemid", new PrintCommand("placeholder for place mid"));
        eventMap.put("placemid", new PrintCommand("placeholder for place mid"));
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

    static PathPlannerTrajectory Place2CableProtectorEngage = PathPlanner.loadPath("Place 2 Cable Protector Engage",
            new PathConstraints(4, 3));
    static PathPlannerTrajectory Place2CableProtector = PathPlanner.loadPath("Place 2 Cable Protector",
            new PathConstraints(4, 3));
    static PathPlannerTrajectory Place2LoadingStationEngage = PathPlanner.loadPath("Place 2 Loading Station Engage",
            new PathConstraints(4, 3));
    static PathPlannerTrajectory Place2LoadingStation = PathPlanner.loadPath("Place 2 Loading Station",
            new PathConstraints(4, 3));
    static PathPlannerTrajectory Place3CableProtector = PathPlanner.loadPath("Place 3 Cable Protector",
            new PathConstraints(4, 3));
    static PathPlannerTrajectory Place3LoadingStation = PathPlanner.loadPath("Place 3 Loading Station",
            new PathConstraints(4, 3));
    static PathPlannerTrajectory PlaceEngageLeave = PathPlanner.loadPath("Place and Engage+", new PathConstraints(4, 3));
    static PathPlannerTrajectory PlaceEngage = PathPlanner.loadPath("Place and Engage", new PathConstraints(4, 3));

    public static Command getAutonomousCommand(PathPlannerTrajectory trajectory) {
        FollowPathWithEvents command = new FollowPathWithEvents(generateSwerveCommand(trajectory),
                trajectory.getMarkers(), eventMap);
        return command;
    }

    private static CustomSwerveControllerCommand generateSwerveCommand(Trajectory trajectory) {
        return new CustomSwerveControllerCommand(
                trajectory,
                RobotContainer.swerveDrive::getPose,
                RobotContainer.swerveDrive.kinematics,
                SwerveXPIDController,
                SwerveYPIDController,
                SwerveThetaPIDController,
                RobotContainer.swerveDrive::setModuleStates,
                RobotContainer.swerveDrive);
    }

    public static class CustomSwerveControllerCommand extends SwerveControllerCommand {

        public CustomSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose,
                SwerveDriveKinematics kinematics, PIDController xController, PIDController yController,
                ProfiledPIDController thetaController, Consumer<SwerveModuleState[]> outputModuleStates,
                SwerveDrive swervedrive) {
            super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates,
                    swervedrive);
        }

        public CustomSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose,
                SwerveDriveKinematics kinematics, PIDController xController, PIDController yController,
                ProfiledPIDController thetaController, Supplier<Rotation2d> desiredRotation,
                Consumer<SwerveModuleState[]> outputModuleStates,
                SwerveDrive swervedrive) {
            super(trajectory, pose, kinematics, xController, yController, thetaController, desiredRotation,
                    outputModuleStates, swervedrive);
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            RobotContainer.swerveDrive.stop();
        }
    }
}
