package frc.robot.commands.Drive;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GoToPoint extends CommandBase {
    private PIDController targetX;
    private PIDController targetY;
    private PIDController targetTheta;

    public GoToPoint(Pose2d targetPose) {
        targetX = new PIDController(0.01, 0, 0);
        targetY = new PIDController(0.01, 0, 0);
        targetTheta = new PIDController(0.01, 0, 0);
        targetX.setSetpoint(targetPose.getX());
        targetY.setSetpoint(targetPose.getY());
        targetTheta.setSetpoint(targetPose.getRotation().getRadians());
        targetX.setTolerance(0.05);
        targetY.setTolerance(0.05);
        targetTheta.setTolerance(0.5);
        this.addRequirements(RobotContainer.swerveDrive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        RobotContainer.swerveDrive.setModuleStates(RobotContainer.swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds(
            targetX.calculate(RobotContainer.swerveDrive.getPose().getX()), 
            targetY.calculate(RobotContainer.swerveDrive.getPose().getY()), 
            targetTheta.calculate(RobotContainer.swerveDrive.getAngle().getRadians() % (Math.PI * 2)))));
    }

    @Override
    public boolean isFinished() {
        return (targetX.atSetpoint() && targetY.atSetpoint() && targetTheta.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
    }
    
}
