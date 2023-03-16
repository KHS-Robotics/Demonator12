package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.arm.ArmControlLength;
import frc.robot.commands.arm.ArmControlPivot;
import frc.robot.commands.drive.CenterSwerveModules;
import frc.robot.commands.drive.balance.BalanceSequence;
import frc.robot.commands.grabber.SetGrabber;
import frc.robot.commands.wrist.WristGoToAngle;

public class AutoRoutines {
    static PathPlannerTrajectory traj = PathPlanner.loadPath("Place + Mobility (Cable Protector)", new PathConstraints(3, 2));
    static CommandBase c1 = RobotContainer.getInstance().swerveAutoBuilder.followPath(traj);

    public static Command getPlace1MobilityCableProtector() {
        return new InstantCommand();
    }

    // public static Command getPlace1MobilityLoadingStation() {
    //     var traj = PathPlanner.loadPath("Place + Mobility (Loading Station)", new PathConstraints(3, 2));
        
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> RobotContainer.swerveDrive.setPose(traj.getInitialPose())),
    //         new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS)),
    //         new SetGrabber(false),
    //         RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH),
    //         getTrajectoryCommand(traj)
    //     );
    // }

    // public static Command getPlace1BalanceSequenceCenter() {
    //     var traj = PathPlanner.loadPath("Place + Engage", new PathConstraints(3, 2));

    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> RobotContainer.swerveDrive.setPose(traj.getInitialPose())),
    //         new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS)),
    //         new SetGrabber(false),
    //         RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH),
    //         getTrajectoryCommand(traj),
    //         new BalanceSequence(0)
    //     );
    // }

    private static Command getTrajectoryCommandOld(Trajectory trajectory) {
        return new SwerveControllerCommand(
            trajectory, 
            RobotContainer.swerveDrive::getPose, 
            RobotContainer.swerveDrive.kinematics, 
            new PIDController(4, 0, 0),
            new PIDController(4, 0, 0),
            new ProfiledPIDController(0.4, 0, 0, new TrapezoidProfile.Constraints(1, 1)),
            RobotContainer.swerveDrive::setModuleStates, 
            RobotContainer.swerveDrive
        );
    }
}
