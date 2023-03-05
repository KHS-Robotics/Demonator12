package frc.robot.commands.wrist;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WristHoldSetpoint extends CommandBase {
    private Rotation2d setpoint;

    public WristHoldSetpoint() {
        this.addRequirements(RobotContainer.wrist);
    }

    @Override
    public void initialize() {
        this.setpoint = RobotContainer.wrist.getAngleSetpoint();
    }

    @Override
    public void execute() {
        RobotContainer.wrist.goToAbsoluteAngle(setpoint);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}