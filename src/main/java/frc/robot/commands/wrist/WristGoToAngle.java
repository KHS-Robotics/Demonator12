package frc.robot.commands.wrist;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WristGoToAngle extends CommandBase {
    private Supplier<Rotation2d> angle;
    private double tolerance;

    public WristGoToAngle(Supplier<Rotation2d> angle, double tolerance) {
        //this.addRequirements(RobotContainer.wrist);
        this.angle = angle;
    }

    public WristGoToAngle(Supplier<Rotation2d> angle) {
        this(angle, 0.05);
    }

    @Override
    public void initialize() {
        //RobotContainer.wrist.goToAngle(angle.get());
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        var setpoint = angle.get().getRadians();
        //var current = RobotContainer.wrist.getRelativeAngle().getRadians();
        //return Math.abs(setpoint - current) < tolerance;
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
