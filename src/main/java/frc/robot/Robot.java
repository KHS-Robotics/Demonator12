// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.LightMode;

/**
 * The VM is configured to automatically run this class, and to call the methods
 * corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command autonmousRoutine;

  /**
   * This method is run when the robot is first started up and should be used for
   * any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    robotContainer = RobotContainer.getInstance();

    Limelight.setLedMode(LightMode.eOff);
  }

  /**
   * This method is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic methods, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This method is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (RobotContainer.grabber != null) {
      RobotContainer.grabber.turnOffGrabSolenoid();
    }
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.arm.armPivotSetpointRadians = RobotContainer.arm.getAngle().getRadians();
    RobotContainer.arm.armLengthSetpoint = RobotContainer.arm.getLength();
    RobotContainer.wrist.setAngleSetpoint(RobotContainer.wrist.getAbsoluteAngle());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // get the auto from the chooser
    // this.autonmousRoutine = robotContainer.getAutoCommand();

    
    // get the auto from the chooser
    var trajectory = robotContainer.getAutoCommand();
    // only run autos that actually have a trajectory to avoid a runtime exception
    this.autonmousRoutine = trajectory != null && !trajectory.isEmpty() ? 
      robotContainer.swerveAutoBuilder.fullAuto(trajectory) : 
      new InstantCommand(() -> DriverStation.reportError("Coudld not get autonomous from the chooser!", false));
    

    // start the auto, if there is one
    if (this.autonmousRoutine != null) {
      this.autonmousRoutine.schedule();
    }
  }

  /** This method is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (!RobotContainer.swerveDrive.isCalibrated) {
      DriverStation.reportError("SWERVE DRIVE NOT CALIBRATED!!!!!", false);
    }
  }

  @Override
  public void teleopInit() {
    //RobotContainer.arm.armPivotSetpointRadians = 0.5;
    //RobotContainer.arm.armLengthSetpoint = Constants.MIN_LENGTH;

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonmousRoutine != null) {
      autonmousRoutine.cancel();
    }

    if (RobotContainer.grabber != null) {
      RobotContainer.grabber.turnOffGrabSolenoid();
    }
  }

  /** This method is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This method is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This method is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This method is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
