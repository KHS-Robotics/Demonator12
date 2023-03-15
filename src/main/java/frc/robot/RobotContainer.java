// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmControlJoystick;
import frc.robot.commands.arm.ArmHoldSetpoint;
import frc.robot.commands.drive.CenterSwerveModules;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.HoldAngleWithXbox;
import frc.robot.commands.drive.balance.BalanceSequence;
import frc.robot.commands.drive.balance.DriveOverThenBalanceSequence;
import frc.robot.commands.grabber.SetGrabber;
import frc.robot.commands.wrist.WristDeltaSetpoint;
import frc.robot.commands.wrist.WristGoToAngle;
import frc.robot.commands.wrist.WristHoldSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.drive.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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

  private static final SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  public SwerveAutoBuilder swerveAutoBuilder;

  /** Gets the selected autonomous command. */
  public List<PathPlannerTrajectory> getAutoTrajectory() {
    return autoChooser.getSelected();
  }

  public static final AHRS navx = new AHRS(Port.kUSB);

  /**
   * Returns the angle or "yaw" of the robot in degrees. CW positive ranging from
   * [-180, 180].
   */
  public static double getRobotYaw() {
    return navx.getYaw();
  }

  /**
   * Returns the pitch angle of the robot in degrees. This tracks the
   * forwards/backwards tilt of the robot.
   */
  public static double getRobotPitch() {
    return navx.getRoll();
  }

  public static double getRobotRoll() {
    return -navx.getPitch();
  }

  public static final Field2d field = new Field2d();
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
  public static final LEDStrip leds = new LEDStrip();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureBindings();
    this.configureAutonmousChooser();
  }

  /** Configures the subsystem's default commands. */
  private void configureSubsystemDefaultCommands() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
    arm.setDefaultCommand(new ArmHoldSetpoint());
    wrist.setDefaultCommand(new WristHoldSetpoint());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
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
    Trigger autoCalibrateTeleop = new Trigger(
        () -> (!swerveDrive.isCalibrated && RobotState.isTeleop() && RobotState.isEnabled()));
    autoCalibrateTeleop.onTrue(new CenterSwerveModules(true));

    // Trigger resetWristEncoderTop = new Trigger(() -> (wrist.getTopTalonTach()));
    // resetWristEncoderTop.onTrue(new InstantCommand(() -> wrist.zeroWrist(Math.PI/2.0), wrist));

    // Trigger resetWristEncoderBottom = new Trigger(() -> (wrist.getTopTalonTach()));
    // resetWristEncoderBottom.onTrue(new InstantCommand(() -> wrist.zeroWrist(-Math.PI/2.0), wrist));
  }

  /** Binds commands to xbox controller buttons. */
  private void configureXboxControllerBindings() {
    Trigger forceCalibrate = driverController.back();
    forceCalibrate.onTrue(new CenterSwerveModules(true));

    Trigger resetNavx = driverController.start();
    resetNavx.onTrue(new InstantCommand(() -> swerveDrive.resetNavx()));

    Trigger resetOdometry = driverController.a();
    resetOdometry.onTrue(new InstantCommand(() -> swerveDrive.resetOdometry()));

    // Trigger placeHigh = driverController.x();
    // placeHigh.onTrue(swerveDrive.goToNode(7, 0).andThen(new
    // ArmControlSetpoint(Field.getNodeCoordinatesFieldRelative(7, 0))));

    //Trigger wristUp = driverController.x();
    //wristUp.onTrue(new WristGoToAngle(() -> new Rotation2d()));

    //Trigger wristDown = driverController.b();
    //wristDown.onTrue(new WristGoToAngle(() -> new Rotation2d(-Math.PI / 4)));

    Trigger setBalanceAngleZero = driverController.pov(0);
    setBalanceAngleZero.onTrue(new ProxyCommand(() -> new BalanceSequence(swerveDrive.getPose().getRotation().getDegrees())));

    Trigger setBalanceAngle180 = driverController.pov(180);
    setBalanceAngle180.onTrue(new ProxyCommand(() -> new BalanceSequence(swerveDrive.getPose().getRotation().getDegrees())));

    Trigger testGoingOverChargeStation = driverController.pov(90);
    testGoingOverChargeStation.onTrue(new DriveOverThenBalanceSequence());

    Trigger holdAngleWhileDriving = driverController.rightBumper();
    holdAngleWhileDriving.whileTrue(new HoldAngleWithXbox());

    Trigger cancelAll = driverController.leftBumper();
    cancelAll.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
  }

  /** Binds commands to the operator box. */
  private void configureOperatorBoxBindings() {
    Trigger leftNode = new Trigger(operatorBox::leftNode);
    leftNode.onTrue(new ProxyCommand(
        () -> swerveDrive.goToNode(Field.aprilTagFromInput(operatorBox.getGrid()), operatorBox.getHeight() * 3)));

    Trigger midNode = new Trigger(operatorBox::cubeNode);
    midNode.onTrue(new ProxyCommand(
        () -> swerveDrive.goToNode(Field.aprilTagFromInput(operatorBox.getGrid()), operatorBox.getHeight() * 3 + 1)));

    Trigger rightNode = new Trigger(operatorBox::rightNode);
    rightNode.onTrue(new ProxyCommand(
        () -> swerveDrive.goToNode(Field.aprilTagFromInput(operatorBox.getGrid()), operatorBox.getHeight() * 3 + 2)));

    Trigger abort = new Trigger(operatorBox::abort);
    abort.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    // Trigger zeroArmPivot = new Trigger(operatorBox::zeroArmPivot);
    // zeroArmPivot.onTrue(new InstantCommand(() -> arm.zeroArmPivot()));

    Trigger zeroArmLength = new Trigger(operatorBox::zeroArmLength);
    zeroArmLength.onTrue(new InstantCommand(() -> arm.zeroArmLength()));

  }

  /** Binds commands to the operator stick. */
  private void configureOperatorStickBindings() {
    Trigger highPos = new Trigger(operatorStick::highPos);
    highPos.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS)));

    Trigger midPos = new Trigger(operatorStick::midPos);
    midPos.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.MID_POS)));

    Trigger lowPos = new Trigger(operatorStick::lowPos);
    lowPos.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpoint(Constants.FLOOR_POS, new Rotation2d())));

    Trigger home = new Trigger(operatorStick::home);
    home.onTrue(RobotContainer.arm.goToPivotLength(Math.toRadians(60), Constants.MIN_LENGTH).andThen(
      new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(135)))));

    Trigger armFlat = new Trigger(operatorStick::stow);
    armFlat.onTrue(RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).andThen(
        new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(80)))));

    Trigger scoreAngle = new Trigger(operatorStick::scoreAngle);
    scoreAngle.onTrue(RobotContainer.arm.goToPivotLength(Math.toRadians(45), Constants.MIN_LENGTH));

    Trigger shelfPos = new Trigger(operatorStick::shelfPos);
    shelfPos.onTrue(RobotContainer.arm.goToSetpoint(Constants.SHELF_POS, new Rotation2d()));

    Trigger wristFlat = new Trigger(operatorStick::wristFlat);
    wristFlat.onTrue(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(0))));

    Trigger wristStepUp = new Trigger(operatorStick::wristStepUp);
    wristStepUp.onTrue(new WristDeltaSetpoint(Rotation2d.fromDegrees(10)));

    Trigger wristStepDown = new Trigger(operatorStick::wristStepDown);
    wristStepDown.onTrue(new WristDeltaSetpoint(Rotation2d.fromDegrees(-10)));

    Trigger wristScoreAngle = new Trigger(operatorStick::wristScoreAngle);
    wristScoreAngle.onTrue(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(45))));

    //Trigger wristDownOverride = new Trigger(operatorStick::wristDownOverride);
    //wristDownOverride.onTrue(new WristGoToAngle(() -> wrist.getRelativeAngle().minus(Rotation2d.fromDegrees(10))));

    //Trigger wristUpOverride = new Trigger(operatorStick::wristUpOverride);
    //wristUpOverride.onTrue(new WristGoToAngle(() -> wrist.getRelativeAngle().plus(Rotation2d.fromDegrees(10))));

    Trigger moveArm = new Trigger(() -> Math.abs(operatorStick.getX()) > 0.05 || Math.abs(operatorStick.getY()) > 0.05);
    moveArm.onTrue(new ArmControlJoystick());

    Trigger grip = new Trigger(() -> (/*operatorBox.coneMode() &&*/ operatorStick.closeClaw()));
    grip.onTrue(new SetGrabber(true));

    Trigger release = new Trigger(() -> (/*operatorBox.cubeMode() ||*/ operatorStick.openClaw()));
    release.onTrue(new SetGrabber(false));

    Trigger outtake = new Trigger(operatorStick::outtake);
    outtake.onTrue(new InstantCommand(() -> grabber.set(1)));
    outtake.onFalse(new InstantCommand(() -> grabber.set(0)));

    Trigger intake = new Trigger(operatorStick::intake);
    intake.onTrue(new InstantCommand(() -> grabber.set(-0.35)));
    intake.onFalse(new InstantCommand(() -> grabber.set(0)));

    // Trigger zeroWrist = new Trigger(operatorStick::zeroWrist);
    // zeroWrist.onTrue(new InstantCommand(() -> RobotContainer.wrist.zeroWrist()));
  }

  /**
   * Configures the autonomous chooser over Network Tables (e.g. Smart Dashboard).
   */
  private void configureAutonmousChooser() {
    swerveAutoBuilder = new SwerveAutoBuilder(
      swerveDrive::getPose,
      swerveDrive::setPose,
      swerveDrive.kinematics,
      new PIDConstants(4, 0, 0), // translation
      new PIDConstants(0.4, 0, 0), // rotation
      swerveDrive::setModuleStates,
      getAutonomousEventMap(),
      true,
      swerveDrive
    );

    autoChooser.setDefaultOption("Nothing", new ArrayList<PathPlannerTrajectory>());

    // dynamically create the options using the PathPlanner paths under "src/main/deploy/pathplanner"
    File ppDirectory = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toFile();
    for (File file : ppDirectory.listFiles()) {
      if (!file.isDirectory() && file.getName().endsWith(".path")) {
        // remove ".path" from the name for PathPlanner
        var pathName = file.getName().replace(".path", "");
        autoChooser.addOption(pathName, PathPlanner.loadPathGroup(pathName, new PathConstraints(2, 3)));
      }
    }

    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData("field", field);

    PPSwerveControllerCommand.setLoggingCallbacks(null, RobotContainer.field::setRobotPose,
        swerveDrive::logTargetChassisSpeeds, null);
  }

  private static final HashMap<String, Command> AutonomousEventMap = new HashMap<>();
  /** Gets the event map for PathPlanner's FollowPathWithEvents. */
  private static HashMap<String, Command> getAutonomousEventMap() {
    if (AutonomousEventMap.isEmpty()) {
        AutonomousEventMap.put("PlaceHigh", new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS)));
        AutonomousEventMap.put("PlaceMid", new PrintCommand("placeholder for place mid"));
        AutonomousEventMap.put("PlaceHybrid", new PrintCommand("placeholder for place hybrid"));
        AutonomousEventMap.put("BalanceFacingAway", new BalanceSequence(0));
        AutonomousEventMap.put("BalanceFacingDriver", new BalanceSequence(180));
        AutonomousEventMap.put("ScoreAngle", RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH));
        AutonomousEventMap.put("Release", new SetGrabber(false));
        AutonomousEventMap.put("Grab", new SetGrabber(true));
        AutonomousEventMap.put("Flat", RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).andThen(
          new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(60)))));
        // AutonomousEventMap.put("Outtake", new InstantCommand(() -> RobotContainer.grabber.set(0.6)));
        // AutonomousEventMap.put("Intake", new InstantCommand(() -> RobotContainer.grabber.set(-0.45)));
    }

    return AutonomousEventMap;
}
}
