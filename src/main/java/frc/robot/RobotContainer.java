// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveSwerveWithXbox;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.drive.SwerveDrive;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    public static NetworkTableInstance networkTable;

    public static final AHRS navx = new AHRS(Port.kUSB);
    public static final PowerDistribution pdp = new PowerDistribution();
    
    public static final SwerveDrive swerveDrive = new SwerveDrive();
    public static final Arm arm = new Arm(RobotMap.ARM_PIVOT, RobotMap.ARM_EXTEND);
    
    public static final CommandXboxController driverController = new CommandXboxController(RobotMap.XBOX_PORT);
    
    SendableChooser<Command> chooser = new SendableChooser<>();
    

    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        networkTable = NetworkTableInstance.getDefault();
        swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
        configureBindings();
        chooser.setDefaultOption("nothing", new PrintCommand("No auto :("));
        chooser.addOption("one piece", new PrintCommand("placeholder for single placement command"));
        chooser.addOption("one piece + engage", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.PlaceEngage));
        chooser.addOption("one piece + engage + leave", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.PlaceEngageLeave));
        chooser.addOption("two piece (cable protector)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place2CableProtector));
        chooser.addOption("two piece (loading station)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place2LoadingStation));
        chooser.addOption("two piece (cable protector) (engage)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place2CableProtectorEngage));
        chooser.addOption("two piece (loading station) (engage)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place2LoadingStation));
        chooser.addOption("three piece (cable protector)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place3CableProtector));
        chooser.addOption("three piece (loading station)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place3LoadingStation));

    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    

    
}
