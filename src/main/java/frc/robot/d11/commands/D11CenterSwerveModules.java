package frc.robot.d11.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.d11.D11RobotContainer;

public class D11CenterSwerveModules extends CommandBase {
    private boolean force;
  
    /**
     * Creates a new CenterSwerveModules.
     */
    public D11CenterSwerveModules(boolean force) {
      this.addRequirements(D11RobotContainer.swerveDrive);
      this.force = force;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      D11RobotContainer.swerveDrive.stop();
      D11RobotContainer.swerveDrive.isCalibrated = !force && D11RobotContainer.swerveDrive.isCalibrated;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (!D11RobotContainer.swerveDrive.isCalibrated) {
        D11RobotContainer.swerveDrive.isCalibrated = D11RobotContainer.swerveDrive.resetEncoders();
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      D11RobotContainer.swerveDrive.stop();
      D11RobotContainer.swerveDrive.isCalibrated = !interrupted;
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return D11RobotContainer.swerveDrive.isCalibrated;
    }
  }
