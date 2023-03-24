package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoPullIn extends CommandBase {
    public AutoPullIn() {
        this.addRequirements(RobotContainer.grabber);
    }

    @Override
    public void initialize() {
        RobotContainer.grabber.set(-0.45);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.grabber.set(0);
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.grabber.getSensor();
    }
}
