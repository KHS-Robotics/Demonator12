// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmControlJoystick;
import frc.robot.commands.arm.ArmHoldSetpoint;
import frc.robot.commands.drive.CenterSwerveModules;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.HoldAngleWithXbox;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.drive.balance.ApproachChargeStation;
import frc.robot.commands.drive.balance.BalanceSequence;
import frc.robot.commands.drive.balance.DriveForward;
import frc.robot.commands.drive.balance.DriveOverThenBalanceSequence;
import frc.robot.commands.grabber.SetGrabber;
import frc.robot.commands.wrist.WristDeltaSetpoint;
import frc.robot.commands.wrist.WristHoldSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.lighting.OldLEDStrip;

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

  private static final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();
  public static SwerveAutoBuilder swerveAutoBuilder;

  /** Gets the selected autonomous command. */
  public AutoRoutine getAutoRoutine() {
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
  public static final OldLEDStrip leds = new OldLEDStrip();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureAutonmousChooser();
    this.configureBindings();
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
    
    //Trigger autoPullIn = new Trigger(() -> operatorBox.cubeMode() && grabber.getSensor());
    //autoPullIn.onTrue(new AutoPullIn());
  }

  /** Binds commands to xbox controller buttons. */
  private void configureXboxControllerBindings() {
    Trigger forceCalibrate = driverController.back();
    forceCalibrate.onTrue(new CenterSwerveModules(true));

    //Trigger resetNavx = driverController.start();
    //resetNavx.onTrue(new InstantCommand(() -> swerveDrive.resetNavx()));

    
    Trigger resetOdometry = driverController.start();
    resetOdometry.onTrue(new InstantCommand(() -> swerveDrive.resetOdometry()));

    Trigger slowDrive = driverController.leftTrigger(0.3);
    slowDrive.onTrue(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 0.75;
      SwerveDrive.kMaxSpeedMetersPerSecond = Math.PI / 2.0;
    }));
    slowDrive.onFalse(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
      SwerveDrive.kMaxSpeedMetersPerSecond = 3.5;
    }));

    Trigger goToClosestNode = driverController.x();
    goToClosestNode.onTrue(new ProxyCommand(() -> swerveDrive.goToClosestNode()));

    Trigger goToSingleSubstation = driverController.y();
    goToSingleSubstation.onTrue(new ProxyCommand(() -> swerveDrive.goToSingleSubstation()));

    //Trigger wristDown = driverController.b();
    //wristDown.onTrue(new WristGoToAngle(() -> new Rotation2d(-Math.PI / 4)));

    Trigger setBalanceAngleZero = driverController.pov(0);
    setBalanceAngleZero.onTrue(new ProxyCommand(() -> new BalanceSequence(swerveDrive.getPose().getRotation().getDegrees())));

    Trigger setBalanceAngle180 = driverController.pov(180);
    setBalanceAngle180.onTrue(new ProxyCommand(() -> new BalanceSequence(swerveDrive.getPose().getRotation().getDegrees())));

    Trigger holdAngleWhileDriving = driverController.rightBumper();
    holdAngleWhileDriving.whileTrue(new HoldAngleWithXbox());

    Trigger cancelAll = driverController.leftBumper();
    cancelAll.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    //Trigger testSequence = driverController.pov(90);
    //testSequence.onTrue(new DriveOverThenBalanceSequence());
  }

  /** Binds commands to the operator box. */
  private void configureOperatorBoxBindings() {
    Trigger leftNode = new Trigger(operatorBox::rightNode); // mirror button bc scoring facing drivers
    leftNode.onTrue(new ProxyCommand(
        () -> swerveDrive.goToNode(Field.aprilTagFromInput(operatorBox.getGrid()), operatorBox.getHeight() * 3)));

    Trigger midNode = new Trigger(operatorBox::cubeNode);
    midNode.onTrue(new ProxyCommand(
        () -> swerveDrive.goToNode(Field.aprilTagFromInput(operatorBox.getGrid()), operatorBox.getHeight() * 3 + 1)));

    Trigger rightNode = new Trigger(operatorBox::leftNode); // mirror button bc scoring facing drivers
    rightNode.onTrue(new ProxyCommand(
        () -> swerveDrive.goToNode(Field.aprilTagFromInput(operatorBox.getGrid()), operatorBox.getHeight() * 3 + 2)));

    Trigger abort = new Trigger(operatorBox::abort);
    abort.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    // Trigger zeroArmPivot = new Trigger(operatorBox::zeroArmPivot);
    // zeroArmPivot.onTrue(new InstantCommand(() -> arm.zeroArmPivot()));

    Trigger highConeKnockedOver = new Trigger(operatorBox::highConeKnockedOver);
    highConeKnockedOver.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpoint(Constants.HIGH_POS_KNOCKED_OVER, Rotation2d.fromRadians(-0.90))));
    
    Trigger midConeKnockedOver = new Trigger(operatorBox::midConeKnockedOver);
    midConeKnockedOver.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpoint(Constants.MID_POS_KNOCKED_OVER, Rotation2d.fromRadians(-0.90))));
  }

  /** Binds commands to the operator stick. */
  private void configureOperatorStickBindings() {
    Trigger highPos = new Trigger(() -> (operatorStick.highPos() && operatorBox.coneMode()));
    highPos.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS)));

    Trigger midPos = new Trigger(() -> (operatorStick.midPos() && operatorBox.coneMode()));
    midPos.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpointScore(Constants.MID_POS)));

    Trigger highPosCube = new Trigger(() -> (operatorStick.highPos() && operatorBox.cubeMode()));
    highPosCube.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpoint(Constants.CUBE_HIGH_POS, new Rotation2d())));

    Trigger midPosCube = new Trigger(() -> (operatorStick.midPos() && operatorBox.cubeMode()));
    midPosCube.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpoint(Constants.CUBE_MID_POS, new Rotation2d())));

    Trigger lowPos = new Trigger(operatorStick::lowPos);
    lowPos.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpoint(Constants.FLOOR_POS, Rotation2d.fromDegrees(0))));

    Trigger home = new Trigger(operatorStick::home);
    home.onTrue(RobotContainer.arm.goToPivotLength(Math.toRadians(60), Constants.MIN_LENGTH).asProxy().andThen(
      new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(150)))));

    Trigger armFlat = new Trigger(operatorStick::stow);
    armFlat.onTrue(RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).asProxy().andThen(
        new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(80)))));

    Trigger stow = new Trigger(operatorStick::scoreAngle);
    stow.onTrue(RobotContainer.arm.goToPivotLength(0.63, Constants.MIN_LENGTH).asProxy().alongWith(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(Math.toDegrees(0.63) + 80)))));

    Trigger shelfPos = new Trigger(operatorStick::shelfPos);
    shelfPos.onTrue(RobotContainer.arm.goToSetpoint(Constants.SHELF_POS, new Rotation2d()));

    Trigger wristFlat = new Trigger(operatorStick::wristFlat);
    wristFlat.onTrue(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(0))));

    Trigger wristStepUp = new Trigger(operatorStick::wristStepUp);
    wristStepUp.onTrue(new WristDeltaSetpoint(Rotation2d.fromDegrees(7)));

    Trigger wristStepDown = new Trigger(operatorStick::wristStepDown);
    wristStepDown.onTrue(new WristDeltaSetpoint(Rotation2d.fromDegrees(-7)));

    Trigger wristScoreAngle = new Trigger(operatorStick::wristScoreAngle);
    wristScoreAngle.onTrue(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(45))));

    Trigger moveArm = new Trigger(() -> Math.abs(operatorStick.getX()) > 0.05 || Math.abs(operatorStick.getY()) > 0.05);
    moveArm.onTrue(new ArmControlJoystick());

    Trigger grip = new Trigger(() -> (/*operatorBox.coneMode() &&*/ operatorStick.openClaw()));
    grip.onTrue(new SetGrabber(true).alongWith(new InstantCommand(() -> grabber.stopWaitingForCone())));

    Trigger release = new Trigger(() -> (/*operatorBox.cubeMode() ||*/ operatorStick.closeClaw()));
    release.onTrue(new SetGrabber(false).alongWith(new InstantCommand(() -> grabber.stopWaitingForCone())));

    Trigger outtake = new Trigger(operatorStick::outtake);
    outtake.onTrue(new InstantCommand(() -> grabber.set(0.3)));
    outtake.onFalse(new InstantCommand(() -> grabber.set(0)));

    Trigger intake = new Trigger(operatorStick::intake);
    intake.onTrue(new InstantCommand(() -> grabber.set(-0.5)));
    intake.onFalse(new InstantCommand(() -> grabber.set(0)));

    Trigger waitForCone = new Trigger(operatorStick::waitForCone);
    waitForCone.onTrue(new InstantCommand(() -> grabber.waitForCone()));

    Trigger coneIn = new Trigger(() -> (grabber.waiting && grabber.getSensor() && operatorBox.coneMode() && RobotState.isTeleop()));
    coneIn.onTrue(new SetGrabber(false)/*.alongWith(new InstantCommand(() -> grabber.stopWaitingForCone()))*/);

    Trigger singleSubstation = new Trigger(operatorStick::singleSubstation);
    singleSubstation.onTrue(new ProxyCommand(() -> RobotContainer.arm.goToSetpoint(Constants.SINGLE_POS, Rotation2d.fromDegrees(35))));

    Trigger autoWaitForCone = new Trigger(() -> (operatorStick.singleSubstation() || operatorStick.shelfPos() || operatorStick.lowPos()) && operatorBox.coneMode());
    autoWaitForCone.onTrue(new InstantCommand(() -> RobotContainer.grabber.waitForCone()));

    Trigger stopWaitingForCone = new Trigger(operatorBox::cubeMode);
    stopWaitingForCone.onTrue(new InstantCommand(() -> RobotContainer.grabber.stopWaitingForCone()));

    Trigger autoRetract = new Trigger(() -> operatorStick.openClaw() && arm.getTranslation().getNorm() > 0.75 && swerveDrive.getPose().getX() < 2.5);
    autoRetract.onTrue(new WaitCommand(0.2).andThen((RobotContainer.arm.goToPivotLength(0.63, Constants.MIN_LENGTH).asProxy()).alongWith(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(Math.toDegrees(0.63) + 80))))));
  }

  /**
   * Configures the autonomous chooser over Network Tables (e.g. Smart Dashboard).
   */
  private void configureAutonmousChooser() {
    swerveAutoBuilder = new SwerveAutoBuilder(
      swerveDrive::getPose,
      swerveDrive::setPose,
      swerveDrive.kinematics,
      new PIDConstants(4, 0, 0.3), // translation
      new PIDConstants(1.8, 0, 0.8), // rotation
      swerveDrive::setModuleStates,
      getAutonomousEventMap(),
      true,
      swerveDrive
    );

    // Do nothing routine
    autoChooser.setDefaultOption("Nothing", new AutoRoutine(new ArrayList<PathPlannerTrajectory>()));

    // dynamically create the options using the PathPlanner paths under "src/main/deploy/pathplanner"
    File ppDirectory = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").toFile();
    for (File file : ppDirectory.listFiles()) {
      if (!file.isDirectory() && file.getName().endsWith(".path")) {
        // remove ".path" from the name for PathPlanner
        var pathName = file.getName().replace(".path", "");
        autoChooser.addOption(pathName, new AutoRoutine(PathPlanner.loadPathGroup(pathName, new PathConstraints(1.4, 2))));
      }
    }

    // "Manually written" autos
    autoChooser.addOption("Place + Mobility + Engage (center)", new AutoRoutine(
      new CenterSwerveModules(false).andThen( // ensure calbirated
      RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS)).andThen( // place high
      new SetGrabber(true).andThen(new WaitCommand(0.3))).andThen( // release, then very briefly wait to drop game piece
      RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).andThen(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(80))))).andThen( // retract
      new DriveOverThenBalanceSequence().deadlineWith(new ArmHoldSetpoint().alongWith(new WristHoldSetpoint()))), // go over + balance
      new Pose2d(1.82, 3.30, Rotation2d.fromDegrees(180))
    ));

    autoChooser.addOption("Place + Mobility + Pickup + Engage", new AutoRoutine(
      new InstantCommand(() -> SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI).andThen(
      new CenterSwerveModules(false)).andThen( // ensure calbirated
      (RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS)).withTimeout(2.5)).andThen( // place high
      new SetGrabber(true).andThen(new WaitCommand(0.3))).andThen( // release, then very briefly wait to drop game piece
      RobotContainer.arm.goToPivotLength(Math.toRadians(0), Constants.MIN_LENGTH).alongWith(new WaitCommand(0.3).andThen((new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(80)))).andThen( // retract, wait 0.3s
      (new ApproachChargeStation(180, true)).finallyDo((inter) -> { if (inter) CommandScheduler.getInstance().cancelAll(); }).withTimeout(2.5)).andThen( // approach charge station
      (new DriveForward(180, true, 1).withTimeout(2.2))).andThen( // drive over charge station
      new RotateToAngle(3).alongWith(RobotContainer.arm.goToSetpoint(Constants.FLOOR_POS, Rotation2d.fromDegrees(0))).alongWith(new InstantCommand(() -> RobotContainer.grabber.set(-0.7)))).andThen( // rotate and lower gripper to pickup cube
      (new DriveForward(0, false, 0.75).withTimeout(0.5))).andThen( // drive into cube with gripper down
      new BalanceSequence(0, true).alongWith(RobotContainer.arm.goToPivotLength(0, Constants.MIN_LENGTH).asProxy().alongWith(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(45))))))))),
      new Pose2d(1.93, 2.17, Rotation2d.fromDegrees(180))
    ));

    autoChooser.addOption("balanceNoArm", new AutoRoutine(new BalanceSequence(0), new Pose2d(1.82, 3.30, Rotation2d.fromDegrees(0))));

    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData("field", field);

    PPSwerveControllerCommand.setLoggingCallbacks(null, RobotContainer.field::setRobotPose,
        swerveDrive::logTargetChassisSpeeds, null);
  }

  private static final HashMap<String, Command> AutonomousEventMap = new HashMap<>();
  /** Gets the event map for PathPlanner's FollowPathWithEvents. */
  private static HashMap<String, Command> getAutonomousEventMap() {
    if (AutonomousEventMap.isEmpty()) {
        AutonomousEventMap.put("CenterSwerveModules", new CenterSwerveModules(false)); 
        AutonomousEventMap.put("PlaceHigh", RobotContainer.arm.goToSetpointScore(Constants.HIGH_POS).withTimeout(3.0)); 
        AutonomousEventMap.put("PlaceHighFast", RobotContainer.arm.goToSetpointScoreFast(Constants.HIGH_POS));
        AutonomousEventMap.put("BalanceFacingAway", new BalanceSequence(0));
        AutonomousEventMap.put("BalanceFacingDriver", new BalanceSequence(180));
        AutonomousEventMap.put("BalanceFacingDriverReversed", new BalanceSequence(180, true));
        AutonomousEventMap.put("DriveOverThenBalance", new DriveOverThenBalanceSequence());
        AutonomousEventMap.put("ScoreAngle", RobotContainer.arm.goToPivotLength(0.75, Constants.MIN_LENGTH).asProxy().withTimeout(2));
        AutonomousEventMap.put("Release", new SetGrabber(true).andThen(new WaitCommand(0.3)));
        AutonomousEventMap.put("Grab", new SetGrabber(false));
        AutonomousEventMap.put("Stow", RobotContainer.arm.goToPivotLength(0.63, Constants.MIN_LENGTH).asProxy().alongWith(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(Math.toDegrees(0.63) + 80)))));
        AutonomousEventMap.put("Flat", RobotContainer.arm.goToPivotLength(0, Constants.MIN_LENGTH).asProxy().alongWith(new InstantCommand(() -> wrist.setAngleSetpoint(Rotation2d.fromDegrees(45)))));
        AutonomousEventMap.put("Hold", new ArmHoldSetpoint().alongWith(new WristHoldSetpoint()));
        AutonomousEventMap.put("Floor", RobotContainer.arm.goToSetpoint(Constants.FLOOR_POS, Rotation2d.fromDegrees(0)));
        AutonomousEventMap.put("Outtake", new InstantCommand(() -> RobotContainer.grabber.set(0.35)));
        AutonomousEventMap.put("OuttakeFast", new InstantCommand(() -> RobotContainer.grabber.set(1)));
        AutonomousEventMap.put("Intake", new InstantCommand(() -> RobotContainer.grabber.set(-0.7)));
        AutonomousEventMap.put("PlaceHighCube", RobotContainer.arm.goToSetpoint(Constants.CUBE_HIGH_POS, new Rotation2d()));
    }

    return AutonomousEventMap;
  }
}
