package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GoToPoint extends CommandBase {
    private PIDController targetX;
    private PIDController targetY;
    private PIDController targetTheta;
    private Pose2d targetPose;

    public GoToPoint(Pose2d targetPose) {
        this.targetPose = targetPose;
        targetX = new PIDController(0.01, 0, 0);
        targetY = new PIDController(0.01, 0, 0);
        targetTheta = new PIDController(0.01, 0, 0);
        targetX.setSetpoint(targetPose.getX());
        targetY.setSetpoint(targetPose.getY());
        targetTheta.setSetpoint(targetPose.getRotation().getRadians());
        this.addRequirements(RobotContainer.swerveDrive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        RobotContainer.swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds(
            targetX.calculate(RobotContainer.swerveDrive.getPose().getX()), 
            targetY.calculate(RobotContainer.swerveDrive.getPose().getY()), 
            targetTheta.calculate(RobotContainer.swerveDrive.getAngle().getRadians() % (Math.PI * 2))));
    }

    @Override
    public boolean isFinished() {
        if (targetPose.getTranslation().getDistance(RobotContainer.swerveDrive.getPose().getTranslation()) < 0.05 && targetPose.getRotation().getRadians() - RobotContainer.swerveDrive.getAngle().getRadians() < 0.1) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}
