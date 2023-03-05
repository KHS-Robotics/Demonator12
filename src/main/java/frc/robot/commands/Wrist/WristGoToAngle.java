package frc.robot.commands.wrist;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WristGoToAngle extends CommandBase {
    private Rotation2d angle;
    private double tolerance;

    public WristGoToAngle(Rotation2d angle, double tolerance) {
        this.addRequirements(RobotContainer.wrist);
        this.angle = angle;
    }

    public WristGoToAngle(Rotation2d angle) {
        this(angle, 0.05);
    }

    @Override
    public void initialize() {
        RobotContainer.wrist.goToAngle(angle);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        var current = RobotContainer.wrist.getRelativeAngle().getRadians();
        var setpoint = angle.getRadians();
        return Math.abs(setpoint - current) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {}
}
