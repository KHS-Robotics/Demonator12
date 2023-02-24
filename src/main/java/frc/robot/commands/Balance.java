package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;


public class Balance extends CommandBase {
    double lastPitch;
    double lastMovingTimestamp;

    public Balance(Rotation2d angle) {
    }

    @Override
    public void initialize() {
        lastPitch = RobotContainer.navx.getPitch();
        lastMovingTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double pitch = RobotContainer.navx.getPitch();
        
        if(Math.abs(pitch - lastPitch) < 0.25) {
            var xSpeed = Math.signum(pitch) * SwerveDrive.kMaxSpeed * 0.1;
            RobotContainer.swerveDrive.drive(xSpeed, 0, 0, true);
        } 
        else if (Math.abs(pitch) > 1){
            lastMovingTimestamp = Timer.getFPGATimestamp();
        }
        lastPitch = pitch;
        
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - lastMovingTimestamp > 1;

    }

    @Override
    public void end(boolean interrupted) {
    }
    
}
