package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WristHoldSetpoint extends CommandBase {
    Rotation2d angle;

    public WristHoldSetpoint() {
        this.addRequirements(RobotContainer.wrist);
    }

    @Override
    public void initialize() {
        this.angle = RobotContainer.wrist.getAngleSetpoint();
    }

    @Override
    public void execute() {
        RobotContainer.wrist.goToAbsoluteAngle(RobotContainer.wrist.getAngleSetpoint());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.wrist.stop();
    }
    
}
