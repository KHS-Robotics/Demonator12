// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.lang3.text.translate.NumericEntityUnescaper.OPTION;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.DriveSwerveWithXbox;
import frc.robot.commands.Arm.ArmControlSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
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
    public static final Arm arm = new Arm(RobotMap.PIVOT_CANCODER, RobotMap.ARM_PIVOT, RobotMap.ARM_EXTEND, 
                                          Constants.ARM_KS, Constants.ARM_KG, Constants.ARM_KV, Constants.ARM_KA, Constants.ARM_P, Constants.ARM_I, Constants.ARM_D, 
                                          Constants.EXTEND_KS, Constants.EXTEND_KV, Constants.EXTEND_KA, Constants.EXTEND_P, Constants.EXTEND_I, Constants.EXTEND_D);
    public static final Wrist wrist = new Wrist();
    
    public static final CommandXboxController driverController = new CommandXboxController(RobotMap.XBOX_PORT);
    public static final OperatorBox operatorBox = new OperatorBox(RobotMap.SWITCHBOX_PORT);
    
    public static final SendableChooser<Command> chooser = new SendableChooser<>();

    public static final Field2d field = new Field2d();
    

    
    
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
        SmartDashboard.putData("field", field);

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
        Trigger forceCalibrate = driverController.back();
        forceCalibrate.onTrue(new CenterSwerveModules(true));

        Trigger resetNavx = driverController.start();
        resetNavx.onTrue(new InstantCommand( () -> swerveDrive.resetNavx()));

        Trigger test = driverController.y();
        test.onTrue(swerveDrive.followTrajectoryCommand(PathPlanner.generatePath(
            new PathConstraints(0.5, 1), 
            new PathPoint(RobotContainer.swerveDrive.getPose().getTranslation(), Rotation2d.fromDegrees(RobotContainer.swerveDrive.getHeading()), Rotation2d.fromDegrees(RobotContainer.swerveDrive.getPose().getRotation().getDegrees())),  // position, heading(direction of travel), holonomic rotation
            new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))), false));

        Trigger resetOdometry = driverController.a();
        resetOdometry.onTrue(new InstantCommand(() -> swerveDrive.resetOdometry()));

        Trigger holdWristFlat = new Trigger(operatorBox::holdWrist);
        holdWristFlat.whileTrue(new InstantCommand(() -> wrist.goToAbsoluteAngle(new Rotation2d(0))));

        Trigger placeHigh = driverController.x();
        placeHigh.onTrue(swerveDrive.goToNode(7, 0).andThen(new ArmControlSetpoint(Field.getNodeCoordinatesFieldRelative(7, 0))));
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
