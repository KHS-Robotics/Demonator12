/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.d11;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.d11.commands.*;
import frc.robot.d11.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class D11RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static NetworkTableEntry id;
  public static final AHRS navx = new AHRS(Port.kUSB);

  public static final PowerDistribution pdp = new PowerDistribution();

  public static final D11SwerveDrive swerveDrive = new D11SwerveDrive();

  public static final XboxController xboxController = new XboxController(D11RobotMap.XBOX_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public D11RobotContainer() {
    swerveDrive.setDefaultCommand(new D11DriveSwerveWithXbox());  
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger autoCalibrateTeleop = new Trigger(() -> (!swerveDrive.isCalibrated && RobotState.isTeleop() && RobotState.isEnabled()));
    autoCalibrateTeleop.onTrue(new D11CenterSwerveModules(true));

    JoystickButton forceCalibrate = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    forceCalibrate.onTrue(new D11CenterSwerveModules(true));

    Trigger holdAngle = new Trigger(xboxController::getAButton);
    holdAngle.whileTrue(new D11HoldAngleWhileDriving());

    Trigger resetNavxButton = new Trigger(xboxController::getStartButton);
    resetNavxButton.onTrue(new InstantCommand(() -> swerveDrive.resetNavx(), swerveDrive));

    Trigger slowDrive = new Trigger(() -> (xboxController.getLeftTriggerAxis() > 0.3 || xboxController.getRightTriggerAxis() > 0.3));
    slowDrive.onTrue(new InstantCommand(() -> {
      D11SwerveDrive.kMaxAngularSpeed = Math.PI;
      D11SwerveDrive.kMaxSpeed = 2;
    }));
    slowDrive.onTrue(new InstantCommand(() -> {
      D11SwerveDrive.kMaxAngularSpeed = 2 * Math.PI;
      D11SwerveDrive.kMaxSpeed = 3.5;
    }));

    
    Trigger goToZero = new Trigger(xboxController::getBButton);
    goToZero.whileTrue(new D11RotateToAngle(180));


    }
}