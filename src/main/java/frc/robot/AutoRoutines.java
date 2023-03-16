package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
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
    public static Command getPlace1MobilityCableProtector() {
        var traj = PathPlanner.loadPath("Place + Mobility (Cable Protector)", new PathConstraints(3, 2));

        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.swerveDrive.setPose(traj.getInitialPose())),
            new InstantCommand(() -> RobotContainer.wrist.setAngleSetpoint(Rotation2d.fromDegrees(45))),
            new ArmControlPivot(0.75).withTimeout(2),
            new ArmControlLength(1).withTimeout(2),
            new InstantCommand(() -> RobotContainer.grabber.release()),
            RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH).withTimeout(2),
            RobotContainer.getInstance().swerveAutoBuilder.followPath(traj)
        );
    }

    public static Command getPlace1MobilityLoadingStation() {
        var traj = PathPlanner.loadPath("Place + Mobility (Loading Station)", new PathConstraints(3, 2));
        
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.swerveDrive.setPose(traj.getInitialPose())),
            new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS).withTimeout(4)).withTimeout(4),
            new InstantCommand(() -> RobotContainer.grabber.release()),
            RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH),
            RobotContainer.getInstance().swerveAutoBuilder.followPath(traj)
        );
    }

    public static Command getPlace1BalanceSequenceCenter() {
        var traj = PathPlanner.loadPath("Place + Engage", new PathConstraints(3, 2));

        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.swerveDrive.setPose(traj.getInitialPose())),
            new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS).withTimeout(4)).withTimeout(4),
            new InstantCommand(() -> RobotContainer.grabber.release()),
            RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH),
            RobotContainer.getInstance().swerveAutoBuilder.followPath(traj),
            new BalanceSequence(0)
        );
    }

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
