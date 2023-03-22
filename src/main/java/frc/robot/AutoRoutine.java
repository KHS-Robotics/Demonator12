package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Wrapper class for auto routines. Supports a manually written command or a path planner trajectory.
 */
public class AutoRoutine {
    public Pose2d startingPose;
    public Command cmdRoutine;
    public List<PathPlannerTrajectory> pathplannerRoutine;

    public AutoRoutine(Command routine, Pose2d startingPose) {
        this.cmdRoutine = routine;
        this.startingPose = startingPose;
    }

    public AutoRoutine(List<PathPlannerTrajectory> routine) {
        this.pathplannerRoutine = routine;
    }

    public boolean isPathPlannerRoutine() {
        return this.pathplannerRoutine != null;
    }

    public boolean isManuallyWrittenRoutine() {
        return !this.isPathPlannerRoutine();
    }
}
