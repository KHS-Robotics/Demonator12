// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Pathing.AutoRoutineBuilder;
import frc.robot.commands.Arm.ArmControlJoystick;
import frc.robot.commands.Arm.ArmHoldSetpoint;
import frc.robot.commands.Drive.CenterSwerveModules;
import frc.robot.commands.Drive.DriveSwerveWithXbox;
import frc.robot.commands.Wrist.WristGoToAngle;
import frc.robot.commands.Wrist.WristHoldSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.drive.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private static RobotContainer instance;
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private static final SendableChooser<Command> autoChooser = new SendableChooser<>();
    /** Gets the selected autonomous command. */
    public Command getAutonomousRoutine() {
        return autoChooser.getSelected();
    }

    public static final Field2d field = new Field2d();
    public static final AHRS navx = new AHRS(Port.kUSB);
    public static final PowerDistribution pdp = new PowerDistribution();
    
    // Human Interface Devices (HIDs)
    public static final CommandXboxController driverController = new CommandXboxController(RobotMap.XBOX_PORT);
    public static final OperatorBox operatorBox = new OperatorBox(RobotMap.SWITCHBOX_PORT);
    public static final OperatorStick operatorStick = new OperatorStick(RobotMap.JOYSTICK_PORT);

    // Subsystems
    public static final SwerveDrive swerveDrive = new SwerveDrive();   
    public static final Arm arm = new Arm();
    public static final Wrist wrist = new Wrist();
    public static final Grabber grabber = new Grabber();
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {
        this.configureSubsystemDefaultCommands();
        this.configureBindings();
        this.configureAutonmousChooser();
    }
    
    /** Configures the subsystem's default commands.  */
    private void configureSubsystemDefaultCommands() {
        swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
        arm.setDefaultCommand(new ArmHoldSetpoint());
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
    private void configureBindings() {
        this.configureAutomatedBindings();
        this.configureXboxControllerBindings();
        this.configureOperatorBoxBindings();
        this.configureOperatorStickBindings();
    }

    /** Automated bindings that happen without pressing any buttons. */
    private void configureAutomatedBindings() {
        Trigger autoCalibrateTeleop = new Trigger(() -> (!swerveDrive.isCalibrated && RobotState.isTeleop() && RobotState.isEnabled()));
        autoCalibrateTeleop.onTrue(new CenterSwerveModules(true));
    }

    /** Binds commands to xbox controller buttons. */
    private void configureXboxControllerBindings() {
        Trigger forceCalibrate = driverController.back();
        forceCalibrate.onTrue(new CenterSwerveModules(true));

        Trigger resetNavx = driverController.start();
        resetNavx.onTrue(new InstantCommand( () -> swerveDrive.resetNavx()));

        Trigger test = driverController.y();
        test.onTrue(swerveDrive.followTrajectoryCommand(AutoRoutineBuilder.Place3LoadingStation, true));

        Trigger resetOdometry = driverController.a();
        resetOdometry.onTrue(new InstantCommand(() -> swerveDrive.resetOdometry()));

        //Trigger placeHigh = driverController.x();
        //placeHigh.onTrue(swerveDrive.goToNode(7, 0).andThen(new ArmControlSetpoint(Field.getNodeCoordinatesFieldRelative(7, 0))));

        //Trigger intake = driverController.leftBumper();
        //intake.onTrue(new InstantCommand(() -> grabber.intake(0.2)));

        //Trigger outtake = driverController.leftBumper();
        //outtake.onTrue(new InstantCommand(() -> grabber.outtake(0.2)));

        Trigger wristUp = driverController.x();
        wristUp.onTrue(new WristGoToAngle(new Rotation2d()));

        Trigger wristDown = driverController.b();
        wristDown.onTrue(new WristGoToAngle(new Rotation2d(-Math.PI / 4)));

        Trigger zeroWrist = driverController.start();
        zeroWrist.onTrue(new InstantCommand(() -> wrist.zeroWrist()));
    }

    /** Binds commands to the operator box. */
    private void configureOperatorBoxBindings() {

    }
    
    /** Binds commands to the operator stick. */
    private void configureOperatorStickBindings() {
        Trigger highPos = new Trigger(operatorStick::highPos);
        highPos.onTrue(RobotContainer.arm.goToSetpoint(Constants.HIGH_POS));
        
        Trigger midPos = new Trigger(operatorStick::midPos);
        midPos.onTrue(RobotContainer.arm.goToSetpoint(Constants.MID_POS));
        
        Trigger lowPos = new Trigger(operatorStick::lowPos);
        lowPos.onTrue(RobotContainer.arm.goToSetpoint(new Translation3d()));

        Trigger home = new Trigger(operatorStick::home);
        home.onTrue(RobotContainer.arm.goToPivotLength(Math.toRadians(60), Constants.MIN_LENGTH).alongWith(
                    new InstantCommand(() -> wrist.goToAngle(new Rotation2d(Math.toRadians(35))))));
        
        Trigger stow = new Trigger(operatorStick::stow);
        stow.onTrue(RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).alongWith(
                    new InstantCommand(() -> wrist.goToAbsoluteAngle(new Rotation2d(Math.toRadians(90))))));

        Trigger scoreAngle = new Trigger(operatorStick::home);
        scoreAngle.onTrue(RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH));

        //Trigger shelfPos = new Trigger(operatorStick::shelfPos);
        //shelfPos.onTrue(RobotContainer.arm.goToSetpoint(new Translation3d(Units.inchesToMeters(20), 0, Units.inchesToMeters(40.81))));

        Trigger wristFlat = new Trigger(operatorStick::wristFlat);
        wristFlat.onTrue(new InstantCommand(() -> wrist.setAngleSetpoint(new Rotation2d())).andThen(new WristHoldSetpoint()));

        Trigger wristStepUp = new Trigger(operatorStick::wristFlat);
        wristStepUp.onTrue(new InstantCommand(() -> wrist.setAngleSetpoint(wrist.getAngleSetpoint().plus(Rotation2d.fromDegrees(10)))).andThen(new WristHoldSetpoint()));

        Trigger wristStepDown = new Trigger(operatorStick::wristFlat);
        wristStepDown.onTrue(new InstantCommand(() -> wrist.setAngleSetpoint(wrist.getAngleSetpoint().minus(Rotation2d.fromDegrees(10)))).andThen(new WristHoldSetpoint()));

        Trigger wristStraightDown = new Trigger(operatorStick::wristFlat);
        wristStraightDown.onTrue(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(-90))).andThen(new WristHoldSetpoint()));

        Trigger wristDownOverride = new Trigger(operatorStick::wristDownOverride);
        wristDownOverride.onTrue(new WristGoToAngle(wrist.getRelativeAngle().minus(Rotation2d.fromDegrees(10))));

        Trigger wristUpOverride = new Trigger(operatorStick::wristDownOverride);
        wristUpOverride.onTrue(new WristGoToAngle(wrist.getRelativeAngle().minus(Rotation2d.fromDegrees(10))));

        Trigger moveArm = new Trigger(() -> operatorStick.getX() > 0.025 || operatorStick.getY() > 0.025);
        moveArm.onTrue(new ArmControlJoystick());
    }

    /** Configures the autonomous chooser over Network Tables (e.g. Smart Dashboard). */
    private void configureAutonmousChooser() {
        autoChooser.setDefaultOption("nothing", new PrintCommand("No auto :("));
        autoChooser.addOption("one piece", new PrintCommand("placeholder for single placement command"));
        autoChooser.addOption("one piece + engage", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.PlaceEngage));
        autoChooser.addOption("one piece + engage + leave", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.PlaceEngageLeave));
        autoChooser.addOption("two piece (cable protector)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place2CableProtector));
        autoChooser.addOption("two piece (loading station)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place2LoadingStation));
        autoChooser.addOption("two piece (cable protector) (engage)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place2CableProtectorEngage));
        autoChooser.addOption("two piece (loading station) (engage)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place2LoadingStation));
        autoChooser.addOption("three piece (cable protector)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place3CableProtector));
        autoChooser.addOption("three piece (loading station)", AutoRoutineBuilder.getAutonomousCommand(AutoRoutineBuilder.Place3LoadingStation));
        SmartDashboard.putData(autoChooser);
        
        SmartDashboard.putData("field", field);
        SmartDashboard.putNumber("xFF", AutoRoutineBuilder.PlaceEngage.sample(0).velocityMetersPerSecond * AutoRoutineBuilder.PlaceEngage.sample(0).poseMeters.getRotation().getCos());
        SmartDashboard.putNumber("yFF", AutoRoutineBuilder.PlaceEngage.sample(0).velocityMetersPerSecond * AutoRoutineBuilder.PlaceEngage.sample(0).poseMeters.getRotation().getSin());
        SmartDashboard.putNumber("desiredX", AutoRoutineBuilder.PlaceEngage.sample(0).poseMeters.getX());
        SmartDashboard.putNumber("curX", swerveDrive.getPose().getX());
        
        PPSwerveControllerCommand.setLoggingCallbacks(null, RobotContainer.field::setRobotPose, swerveDrive::logTargetChassisSpeeds, null);
    }
}
