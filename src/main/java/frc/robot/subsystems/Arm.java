package frc.robot.subsystems;

import java.util.HashMap;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.arm.ArmControlLength;
import frc.robot.commands.arm.ArmControlPivot;
import frc.robot.commands.arm.ArmControlPivotLength;
import frc.robot.commands.wrist.WaitUntilUp;
import frc.robot.commands.wrist.WristGoToAngle;
import frc.robot.commands.wrist.WristGoToSetpoint;
import frc.robot.commands.wrist.WristHoldAngle;
import frc.robot.commands.wrist.WristHoldSetpoint;

public class Arm extends SubsystemBase {
  public Translation3d armTranslaton = new Translation3d();

  private final CANSparkMax pivotMotor;
  private final CANCoder pivotCANCoder;

  private final CANSparkMax extendMotor;
  private final RelativeEncoder extendEncoder;

  private final ArmFeedforward armFeedForward;
  private final PIDController armPID;
  public double armPivotSetpointRadians = 0.5, armLengthSetpoint = Constants.MIN_LENGTH;
  public TrapezoidProfile.State pivotSetpoint = new TrapezoidProfile.State();
  public TrapezoidProfile.State lengthSetpoint = new TrapezoidProfile.State();

  private final SimpleMotorFeedforward extendFeedForward;
  private final PIDController extendPID;
  private double kAL, kDt = 0.02, kSpring;

  TrapezoidProfile.Constraints pivotConstraints;
  TrapezoidProfile.Constraints extendConstraints;

  Position position;

  HashMap<Position, ControlMode> highMap, midMap, stowMap, flatMap, floorMap, shelfMap, singleMap, homeMap, highKnockedMap, midKnockedMap;
  HashMap<Position, HashMap<Position, ControlMode>> locationMap;

  public Arm() {
    pivotMotor = new CANSparkMax(RobotMap.ARM_PIVOT, MotorType.kBrushless);
    pivotMotor.setInverted(true);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    extendMotor = new CANSparkMax(RobotMap.ARM_EXTEND, MotorType.kBrushless);
    extendMotor.setIdleMode(IdleMode.kBrake);

    pivotCANCoder = new CANCoder(RobotMap.ARM_PIVOT_CANCODER);
    extendEncoder = extendMotor.getEncoder();
    extendEncoder.setPositionConversionFactor(Units.inchesToMeters(Math.PI) / 3.2); // 1" diameter spool, 3.2:1
                                                                                    // ratio
    extendEncoder.setVelocityConversionFactor(Units.inchesToMeters(Math.PI) / (3.2 * 60));

    armFeedForward = new ArmFeedforward(Constants.ARM_KS, Constants.ARM_KG, Constants.ARM_KV, Constants.ARM_KA);
    armPID = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);

    extendFeedForward = new SimpleMotorFeedforward(Constants.EXTEND_KS, Constants.EXTEND_KV, Constants.EXTEND_KA);
    extendPID = new PIDController(Constants.EXTEND_P, Constants.EXTEND_I, Constants.EXTEND_D);

    pivotConstraints = new TrapezoidProfile.Constraints(2, 3);
    extendConstraints = new TrapezoidProfile.Constraints(3, 3);

    position = Position.HOME;
    fillHashMaps();


    this.kAL = 1.22566276313;
    this.kDt = 0.02; // 20 ms assumed for control loops
    this.kSpring = 25;
  }

  public void resetPivotPID() {
    armPID.reset();
  }

  public void resetExtendPID() {
    extendPID.reset();
  }

  @Override
  public void periodic() {
    var translation = getTranslation();
    SmartDashboard.putNumber("ArmPivotSetpoint", this.armPivotSetpointRadians);
    SmartDashboard.putNumber("ArmPivot", this.getAngle().getRadians());
    SmartDashboard.putNumber("ArmLength", getLength());
    SmartDashboard.putBoolean("isLegalH", isLegalHeight(translation));
    SmartDashboard.putBoolean("isLegalE", isLegalExtension(translation));

    SmartDashboard.putNumber("ExtendArmSetpoint", lengthSetpoint.position);
    SmartDashboard.putNumber("ExtendArmPosition", getLength());
  }

  // converts point from robot relative to arm relative
  public Translation3d toArmRelative(Translation3d robotRelative) {
    return robotRelative.minus(Constants.ARMOFFSET);
  }

  // converts point from arm relative to robot relative
  public Translation3d toRobotRelative(Translation3d armRelative) {
    return armRelative.plus(Constants.ARMOFFSET);
  }

  // extends the arm
  public void setLength(double length) {
    TrapezoidProfile profile = new TrapezoidProfile(extendConstraints, new TrapezoidProfile.State(length, 0),
        lengthSetpoint);
    lengthSetpoint = profile.calculate(kDt);
    SmartDashboard.putNumber("ExtendSetpointPosition", lengthSetpoint.position);
    // setLengthV(lengthSetpoint.velocity);
    extendMotor.setVoltage(
        calcLengthV(lengthSetpoint.velocity) + extendPID.calculate(getLength(), lengthSetpoint.position));
  }

  // gets the arm extension
  public double getLength() {
    return extendEncoder.getPosition() + Constants.MIN_LENGTH;
  }

  public void setLengthV(double vLength) {
    var voltage = extendFeedForward.calculate(vLength) //+ extendPID.calculate(getLengthV(), vLength)
        + Constants.EXTEND_KG * Math.sin(this.getAngle().getRadians());
    extendMotor.setVoltage(voltage);
    SmartDashboard.putNumber("ExtendVoltage", voltage);
  }

  public double calcLengthV(double vLength) {
    var voltage = extendFeedForward.calculate(vLength)
        + Constants.EXTEND_KG * Math.sin(this.getAngle().getRadians()) + Constants.EXTEND_KSPRING;
    // extendMotor.setVoltage(voltage);
    SmartDashboard.putNumber("ExtendVoltage", voltage);
    return voltage;
  }

  public double getLengthV() {
    return extendEncoder.getVelocity();
  }

  // gets the arm angle
  public Rotation2d getAngle() {
    return new Rotation2d(Math.toRadians(pivotCANCoder.getAbsolutePosition()));
  }

  public void setAngleV(double vAngle) {
    var voltage = calcVoltagePivot(vAngle);
    pivotMotor.setVoltage(voltage);
    SmartDashboard.putNumber("PivotVoltage", voltage);
  }

  public double getAngleV() {
    return Math.toRadians(pivotCANCoder.getVelocity());
  }

  // takes in the position, vel, and accel setpoints, outputs the voltage for
  // telescoping arm (rad, rad/s, rad/s^2)
  public double calcVoltagePivot(double vAngle) {
    double accel = 0;
    var gravityTerm = (getLength() * Constants.ARM_KG * Math.cos(getAngle().getRadians() + vAngle * kDt));
    SmartDashboard.putNumber("ArmGravityTerm", gravityTerm);
    return calcSpringVoltage(getAngle().getRadians()) + Constants.ARM_KS * Math.signum(vAngle)
        + Constants.ARM_KV * vAngle + gravityTerm + Math.pow(getLength(), 2) * kAL * accel;
  }

  public double calcSpringVoltage(double position) {
    double voltagePerTorque = 0.41862 / 344; // VOLTAGE TO HOLD ARM DIVIDED BY TORQUE TO HOLD IN IN-LBS
    double force = kSpring;
    double length = 15;
    double height = Units.metersToInches(Constants.ARMOFFSET.getZ()) - 5;
    double armAngle = getAngle().getRadians();
    double forceAngle = Math.atan2(height - (length * Math.sin(armAngle)), length * Math.cos(armAngle) - 6);
    SmartDashboard.putNumber("forceAngle", forceAngle);
    double torque = force * Math.sin(forceAngle) * length * Math.cos(armAngle);
    return -torque * voltagePerTorque;
  }

  public void setAngle(double angle) {
    TrapezoidProfile profile = new TrapezoidProfile(pivotConstraints, new TrapezoidProfile.State(angle, 0),
        pivotSetpoint);
    pivotSetpoint = profile.calculate(kDt);
    SmartDashboard.putNumber("PivotSetpointPosition", pivotSetpoint.position);

    var armPidOutput = armPID.calculate(getAngle().getRadians(), pivotSetpoint.position);
    pivotMotor.setVoltage(calcAngleV(pivotSetpoint.velocity) + armPidOutput);
  }

  public double calcAngleV(double vAngle) {
    var voltage = calcVoltagePivot(vAngle);
    return voltage;
  }


  // returns the required rotation to go to a setpoint in degrees (arm relative)
  public Rotation2d rotToPointAR(Translation3d target) {

    // defines a new translation3d with no y offset (side to side)
    Translation3d targetXZ = new Translation3d(target.getX(), 0.0, target.getZ());

    // angle to pivot to in degrees (rotating up is positive)
    Rotation2d angle = new Rotation2d(targetXZ.getX(), targetXZ.getZ());

    return angle;
  }

  // returns the required rotation to go to a setpoint in degrees (robot relative)
  public Rotation2d rotToPoint(Translation3d target) {
    return rotToPointAR(toArmRelative(target));
  }

  // returns the required length to go to a setpoint (arm relative)
  public double lengthToPointAR(Translation3d target) {
    // defines a new translation3d with no y offset (side to side)
    Translation2d targetXZ = new Translation2d(target.getX(), target.getZ());

    // calculates distance in the XZ plane (arm length)
    double length = targetXZ.getNorm();

    return length;
  }

  // returns the required length to go to a setpoint (robot relative)
  public double lengthToPoint(Translation3d target) {
    return lengthToPointAR(toArmRelative(target));
  }

  public boolean isLegalExtension(Translation3d target) {
    var x = this.getFurthestPoint(target).getX(); 
    var limit = Units.inchesToMeters(59.5); // 45 IN + 16 IN (HALF OF CHASSIS);
    SmartDashboard.putNumber("ArmX-Inches", Units.metersToInches(x));
    return x < limit;
  }

  public boolean isLegalHeight(Translation3d target) {
    // left some inches since we shouldnt go this high ever (limit is 72" instead of
    // legal 78")
    var y = this.getFurthestPoint(target).getY();
    var limit = Units.inchesToMeters(72);
    SmartDashboard.putNumber("ArmY-Inches", Units.metersToInches(y));
    return y < limit;
  }

  public boolean isFurther(Translation3d target) {
    return getTranslation().getNorm() < target.getNorm();
  }

  //goes to a setpoint while avoiding collision. It does this by first turning the wrist, then if going further it pivots then extend. If retracting it first retracts then pivots.
  public Command goToSetpoint(Translation3d target, Rotation2d wristAngle) {
    var wristTranslation = new Translation3d(Constants.GRIPPERHOLDDISTANCE,
    new Rotation3d(0, -wristAngle.getRadians(), 0));
    target = target.minus(wristTranslation);
    double rotToPoint = rotToPoint(target).getRadians();
    double lengthToPoint = lengthToPoint(target);
      
    armTranslaton = target;


    var command = new SequentialCommandGroup();
    if (isFurther(target)) {
      //wrist, then pivot, then length
      command = new SequentialCommandGroup(
          (new ArmControlPivot(rotToPoint).asProxy().alongWith(new WristGoToSetpoint(wristAngle).asProxy())).andThen(
          new ArmControlLength(lengthToPoint).asProxy()));
    } else {
      //wrist, then length, then pivot
      command = new SequentialCommandGroup(
          (new ArmControlLength(lengthToPoint).asProxy().alongWith(new WristGoToSetpoint(wristAngle).asProxy())).andThen(
          new ArmControlPivot(rotToPoint).asProxy()));
    }
    return command;
  }

  //goes to a setpoint while avoiding collision. It does this by first turning the wrist, then if going further it pivots then extend. If retracting it first retracts then pivots.
  public Command goToSetpointAuto(Translation3d target, Rotation2d wristAngle) {
    var wristTranslation = new Translation3d(Constants.GRIPPERHOLDDISTANCE,
    new Rotation3d(0, -wristAngle.getRadians(), 0));
    target = target.minus(wristTranslation);
    double rotToPoint = rotToPoint(target).getRadians();
    double lengthToPoint = lengthToPoint(target);
      
    armTranslaton = target;


    var command = new SequentialCommandGroup();
    if (isFurther(target)) {
      //wrist, then pivot, then length
      command = new SequentialCommandGroup(
          new InstantCommand(() -> RobotContainer.wrist.setAngleSetpoint(wristAngle)),
          (new ArmControlPivot(rotToPoint).andThen(
          new ArmControlLength(lengthToPoint))));
    } else {
      //wrist, then length, then pivot
      command = new SequentialCommandGroup(
          new InstantCommand(() -> RobotContainer.wrist.setAngleSetpoint(wristAngle)),
          (new ArmControlLength(lengthToPoint)).andThen(
          new ArmControlPivot(rotToPoint)));
    }
    return command;
  }

  /*goes to a setpoint while still trying to avoid collision (might be a bit risky). 
  If extending and the arm is pivoted higher than it needs to be, it should perform all 3 commands in parrallel
  If retracting and the arm is pivoted lower than it needs to be it should perform all 3 commands in parrallel 
  Otherwise it should return a command similar to the normal go to setpoint (i did change this a little by making the wrist
  move in parrallel with the arm*/
  public Command goToSetpointFast(Translation3d target, Rotation2d wristAngle) {
    RobotContainer.wrist.setAngleSetpoint(wristAngle);

    var wristTranslation = new Translation3d(Constants.GRIPPERHOLDDISTANCE,
    new Rotation3d(0, -wristAngle.getRadians(), 0));

    target = target.minus(wristTranslation);
    armTranslaton = target;

    double rotToPoint = rotToPoint(target).getRadians();
    double lengthToPoint = lengthToPoint(target);
      
    boolean isPivotedHigher = getAngle().getRadians() > rotToPoint;

    Command command;
    if (isFurther(target)) {
      if(isPivotedHigher) {
        // all in parallel
        command = new ParallelDeadlineGroup(
        new ArmControlPivotLength(rotToPoint, lengthToPoint).asProxy(),
        new WristHoldSetpoint().asProxy());
      } else {
      return goToSetpoint(target, wristAngle);
      }
    } else {
      if(!isPivotedHigher) {
        // all in parallel
        command = new ParallelDeadlineGroup(
          new ArmControlPivotLength(rotToPoint, lengthToPoint).asProxy(),
          new WristHoldSetpoint().asProxy());
      } else {
      return goToSetpoint(target, wristAngle);
      }
    }
    return command;
  }

  public Command goToSetpointScore(Translation3d target) {
    return goToSetpoint(target, Rotation2d.fromDegrees(40));
  }

  public Command goToSetpointScoreFast(Translation3d target) {
    return goToSetpointFast(target, Rotation2d.fromDegrees(40));
  }

  public Command goToSetpointScoreCube(Translation3d target) {
    return goToSetpointScore(target).andThen(new InstantCommand(() -> RobotContainer.wrist.setAngleSetpoint(Rotation2d.fromDegrees(-25))));
  }

  public Command goToSetpoint(Translation3d target) {
    return goToSetpoint(target, RobotContainer.wrist.getAbsoluteAngle());
  }

  public Command goToPivotLength(double pivot, double length) {
      Command command = new SequentialCommandGroup(new ArmControlLength(length).asProxy(), new ArmControlPivot(pivot).asProxy());
    return command;
  }

  public Translation3d getTranslation() {
    return new Translation3d(getLength(), new Rotation3d(0, -getAngle().getRadians(), 0)).plus(Constants.ARMOFFSET);//.plus(new Translation3d(Constants.GRIPPERLENGTH, new Rotation3d(0, -RobotContainer.wrist.getAbsoluteAngle().getRadians(), 0)));
  }

  private Translation2d getFurthestPoint(Translation3d target) {
    Translation2d targetXZ = new Translation2d(target.getX(), target.getZ());
    Translation2d gripper = new Translation2d(Constants.GRIPPERLENGTH,
        targetXZ.getAngle().plus(RobotContainer.wrist.getRelativeAngle()));
    return targetXZ.plus(gripper);
  }

  public void zeroArmLength() {
    extendEncoder.setPosition(0);
  }
  
  public enum Position {
    HIGH,
    MID,
    STOW,
    FLAT,
    FLOOR,
    SHELF,
    SINGLE,
    HOME,
    HIGH_KNOCKED,
    MID_KNOCKED
  }

  enum ControlMode {
    PARALLEL,
    WRISTRELRETRACT,
    WRISTRELEXTEND,
    FLOORUP
  }

  public void fillHashMaps() {
    setHashMapParallel(highMap);
    setHashMapParallel(midMap);
    setHashMapParallel(stowMap);
    setHashMapParallel(flatMap);
    setHashMapParallel(floorMap);
    setHashMapParallel(shelfMap);
    setHashMapParallel(singleMap);
    setHashMapParallel(homeMap);
    setHashMapParallel(highKnockedMap);
    setHashMapParallel(midKnockedMap);

    highMap.replace(Position.FLAT, ControlMode.WRISTRELRETRACT);
    highMap.replace(Position.FLOOR, ControlMode.WRISTRELRETRACT);
    highMap.replace(Position.SHELF, ControlMode.WRISTRELRETRACT);
    highMap.replace(Position.SINGLE, ControlMode.WRISTRELRETRACT);
    highMap.replace(Position.HIGH_KNOCKED, ControlMode.WRISTRELEXTEND);
    highMap.replace(Position.MID_KNOCKED, ControlMode.WRISTRELEXTEND);

    midMap = highMap;

    stowMap.put(Position.SHELF, ControlMode.WRISTRELEXTEND);
    
    flatMap.put(Position.HIGH, ControlMode.WRISTRELEXTEND);
    flatMap.put(Position.MID, ControlMode.WRISTRELEXTEND);
    flatMap.put(Position.SHELF, ControlMode.WRISTRELEXTEND);
    flatMap.put(Position.HIGH_KNOCKED, ControlMode.WRISTRELEXTEND);
    flatMap.put(Position.MID_KNOCKED, ControlMode.WRISTRELEXTEND);

    shelfMap.put(Position.HIGH, ControlMode.WRISTRELEXTEND);
    shelfMap.put(Position.MID, ControlMode.WRISTRELRETRACT);
    shelfMap.put(Position.STOW, ControlMode.WRISTRELRETRACT);
    shelfMap.put(Position.FLAT, ControlMode.WRISTRELRETRACT);
    shelfMap.put(Position.FLOOR, ControlMode.WRISTRELRETRACT);
    shelfMap.put(Position.HIGH_KNOCKED, ControlMode.WRISTRELEXTEND);
    shelfMap.put(Position.MID_KNOCKED, ControlMode.WRISTRELEXTEND);

    singleMap.put(Position.HIGH, ControlMode.WRISTRELEXTEND);
    singleMap.put(Position.MID, ControlMode.WRISTRELEXTEND);
    singleMap.put(Position.HIGH_KNOCKED, ControlMode.WRISTRELEXTEND);
    singleMap.put(Position.MID_KNOCKED, ControlMode.WRISTRELEXTEND);

    //home is all parallel

    highKnockedMap.put(Position.MID, ControlMode.WRISTRELRETRACT);
    highKnockedMap.put(Position.FLAT, ControlMode.WRISTRELRETRACT);
    highKnockedMap.put(Position.FLOOR, ControlMode.WRISTRELRETRACT);
    highKnockedMap.put(Position.SHELF, ControlMode.WRISTRELRETRACT);

    midKnockedMap.put(Position.MID, ControlMode.WRISTRELRETRACT);
    midKnockedMap.put(Position.FLAT, ControlMode.WRISTRELRETRACT);
    midKnockedMap.put(Position.FLOOR, ControlMode.WRISTRELRETRACT);
    midKnockedMap.put(Position.SHELF, ControlMode.WRISTRELEXTEND);

    floorMap.put(Position.HIGH, ControlMode.FLOORUP);
    floorMap.put(Position.MID, ControlMode.FLOORUP);
    floorMap.put(Position.SINGLE, ControlMode.FLOORUP);
    floorMap.put(Position.HIGH_KNOCKED, ControlMode.FLOORUP);
    floorMap.put(Position.MID_KNOCKED, ControlMode.FLOORUP);
    floorMap.put(Position.STOW, ControlMode.WRISTRELEXTEND);
    floorMap.put(Position.FLAT, ControlMode.WRISTRELEXTEND);
    floorMap.put(Position.SHELF, ControlMode.WRISTRELEXTEND);

    locationMap.put(Position.HIGH, highMap);
    locationMap.put(Position.MID, midMap);
    locationMap.put(Position.STOW, stowMap);
    locationMap.put(Position.FLAT, flatMap);
    locationMap.put(Position.FLOOR, floorMap);
    locationMap.put(Position.SHELF, shelfMap);
    locationMap.put(Position.SINGLE, singleMap);
    locationMap.put(Position.HOME, homeMap);
    locationMap.put(Position.HIGH_KNOCKED, highKnockedMap);
    locationMap.put(Position.MID_KNOCKED, midKnockedMap);

  }

  public void setHashMapParallel(HashMap<Position, ControlMode> hashmap) {
    hashmap.put(Position.HIGH, ControlMode.PARALLEL);
    hashmap.put(Position.MID, ControlMode.PARALLEL);
    hashmap.put(Position.STOW, ControlMode.PARALLEL);
    hashmap.put(Position.FLAT, ControlMode.PARALLEL);
    hashmap.put(Position.FLOOR, ControlMode.PARALLEL);
    hashmap.put(Position.SHELF, ControlMode.PARALLEL);
    hashmap.put(Position.SINGLE, ControlMode.PARALLEL);
    hashmap.put(Position.HOME, ControlMode.PARALLEL);
    hashmap.put(Position.HIGH_KNOCKED, ControlMode.PARALLEL);
    hashmap.put(Position.MID_KNOCKED, ControlMode.PARALLEL);
  }

  
  public Command goToLocationUniversal(Position desiredPosition, Translation3d target, Rotation2d wristAngle) {
    var wristTranslation = new Translation3d(Constants.GRIPPERHOLDDISTANCE,
    new Rotation3d(0, -wristAngle.getRadians(), 0));
    target = target.minus(wristTranslation);
    double rotToPoint = rotToPoint(target).getRadians();
    double lengthToPoint = lengthToPoint(target);

    return goToLocationUniversal(desiredPosition, rotToPoint, lengthToPoint, wristAngle);
  }

  public Command goToLocationUniversal(Position desiredPosition, double pivot, double length, Rotation2d wristAngle) {
    var controlModeMap = locationMap.get(position);
    ControlMode controlMode = controlModeMap.get(desiredPosition);
    position = desiredPosition;

    Command sequence;
    switch (controlMode) {
      case PARALLEL:
      //runs pivot and length and wrist control (relative) at the same time
        sequence =  (new ArmControlPivotLength(pivot, length).asProxy()).deadlineWith((new WristGoToAngle(wristAngle, Rotation2d.fromRadians(pivot)).asProxy()).andThen(new WristHoldAngle().asProxy()));
        break;
      case WRISTRELRETRACT:
      //runs length and wrist control at the same time, once both are done, it will move on to pivot
        sequence = ((new ArmControlLength(length).asProxy()).deadlineWith((new WristGoToAngle(wristAngle, Rotation2d.fromRadians(pivot)).asProxy()).andThen(new WristHoldAngle().asProxy()))).andThen((new ArmControlPivot(pivot).asProxy()).deadlineWith(new WristHoldAngle().asProxy()));
        break;
      case WRISTRELEXTEND:
      //runs pivot and wrist control at the same time, then extends
        sequence = ((new ArmControlPivot(pivot).asProxy()).deadlineWith((new WristGoToAngle(wristAngle, Rotation2d.fromRadians(pivot)).asProxy()).andThen(new WristHoldAngle().asProxy()))).andThen((new ArmControlLength(length).asProxy()).deadlineWith(new WristHoldAngle().asProxy()));
        break;
      case FLOORUP:
      //runs pivot, once it is above -10 degrees or so it will start moving the wrist with relative control (as to not hit the floor) after this it should run length
        sequence = ((new ArmControlPivot(pivot).asProxy()).deadlineWith((new WaitUntilUp().asProxy()).andThen((new WristGoToAngle(wristAngle, Rotation2d.fromRadians(pivot)).asProxy()).andThen(new WristHoldAngle().asProxy())))).andThen((new ArmControlLength(length).asProxy()).deadlineWith(new WristHoldAngle().asProxy()));
        break;
      default: 
        sequence = new PrintCommand("DEFAULT CASE");
    }
    return sequence;

  }

  
}