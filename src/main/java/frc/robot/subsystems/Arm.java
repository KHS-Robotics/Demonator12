package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OperatorStick;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.arm.ArmControlLength;
import frc.robot.commands.arm.ArmControlPivot;


public class Arm extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANCoder pivotCANCoder;

    private final CANSparkMax extendMotor;
    private final RelativeEncoder extendEncoder;

    private final ArmFeedforward armFeedFoward;
    private final PIDController armPID;
    public double armPivotSetpointRadians = 0.5, armLengthSetpoint = Constants.MIN_LENGTH;
    public TrapezoidProfile.State pivotSetpoint = new TrapezoidProfile.State();
    public TrapezoidProfile.State lengthSetpoint = new TrapezoidProfile.State();

    private final SimpleMotorFeedforward extendFeedFoward;
    private final PIDController extendPID;
    private double kGL, kAL, kDt, kSpring;

    TrapezoidProfile.Constraints armConstraints;

    public Arm () {
        pivotMotor = new CANSparkMax(RobotMap.ARM_PIVOT, MotorType.kBrushless);
        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        extendMotor = new CANSparkMax(RobotMap.ARM_EXTEND, MotorType.kBrushless);
        extendMotor.setIdleMode(IdleMode.kBrake);

        pivotCANCoder = new CANCoder(RobotMap.ARM_PIVOT_CANCODER);
        extendEncoder = extendMotor.getEncoder();
        extendEncoder.setPositionConversionFactor(Units.inchesToMeters(Math.PI) / 3.2); // 1" diameter spool, 3.2:1 ratio
        extendEncoder.setVelocityConversionFactor(Units.inchesToMeters(Math.PI) / (3.2 * 60));

        armFeedFoward = new ArmFeedforward(Constants.ARM_KS, Constants.ARM_KG, Constants.ARM_KV, Constants.ARM_KA);
        armPID = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        
        extendFeedFoward = new SimpleMotorFeedforward(Constants.EXTEND_KS, Constants.EXTEND_KV, Constants.EXTEND_KA);
        extendPID = new PIDController(Constants.EXTEND_P, Constants.EXTEND_I, Constants.EXTEND_D);

        armConstraints = new TrapezoidProfile.Constraints(1, 1);

        this.kGL = 0;
        this.kAL = 0;
        this.kDt = 0.02; // 20 ms assumed for control loops
    }

    @Override
    public void periodic() {
        var translation = getTranslation();
        SmartDashboard.putNumber("ArmPivot", this.getAngle().getDegrees());
        SmartDashboard.putNumber("PivotJoystick", RobotContainer.operatorStick.getPitchSpeed());
        SmartDashboard.putNumber("PivotV", Math.toRadians(pivotCANCoder.getVelocity()));
        SmartDashboard.putBoolean("isLegalH", isLegalHeight(translation));
        SmartDashboard.putBoolean("isLegalE", isLegalExtension(translation));
        SmartDashboard.putNumber("translationx", translation.getX());
        SmartDashboard.putNumber("translationy", translation.getY());
        SmartDashboard.putNumber("ExtendArmPosition", getLength());
        SmartDashboard.putNumber("ExtendNeoPosition", extendEncoder.getPosition());
        SmartDashboard.putNumber("ExtendVelocity", extendEncoder.getVelocity());
        SmartDashboard.putNumber("ExtendJoystick", RobotContainer.operatorStick.getExtendSpeed());
    }

    //converts point from robot relative to arm relative
    public Translation3d toArmRelative(Translation3d robotRelative) {
        return robotRelative.minus(Constants.ARMOFFSET);
    }

    //converts point from arm relative to robot relative
    public Translation3d toRobotRelative(Translation3d armRelative) {
        return armRelative.plus(Constants.ARMOFFSET);
    }
    
    //extends the arm
    public void setLength(double length) {
        TrapezoidProfile profile = new TrapezoidProfile(armConstraints, new TrapezoidProfile.State(length, 0), lengthSetpoint);
        lengthSetpoint = profile.calculate(kDt);
        setLengthV(lengthSetpoint.velocity);
    }

    //gets the arm extension
    public double getLength() {
        return extendEncoder.getPosition() + Constants.MIN_LENGTH;
    }

    public void setLengthV(double vLength) {
        var voltage = MathUtil.clamp(extendFeedFoward.calculate(vLength) + extendPID.calculate(getLengthV(), vLength), -3, 6);
        extendMotor.setVoltage(voltage);
        SmartDashboard.putNumber("ExtendVoltage", voltage);
    }

    public double getLengthV() {
        return extendEncoder.getVelocity();
    }

    //gets the arm angle
    public Rotation2d getAngle() {
        return new Rotation2d(Math.toRadians(pivotCANCoder.getAbsolutePosition()));
    }

    public void setAngleV(double vAngle) {
        var voltage = MathUtil.clamp(armFeedFoward.calculate(getAngle().getRadians() + vAngle * kDt, vAngle) + armPID.calculate(getAngleV(), vAngle), -5, 8);
        pivotMotor.setVoltage(voltage);
        SmartDashboard.putNumber("PivotVoltage", voltage);
    }

    public double getAngleV() {
        return Math.toRadians(pivotCANCoder.getVelocity());
    }

    //takes in the position, vel, and accel setpoints, outputs the voltage for telescoping arm (rad, rad/s, rad/s^2)
    public double calcVoltagePivot(double position, double velocity, double accel) {
        return Constants.ARM_KS * Math.signum(velocity) + Constants.ARM_KV * velocity + (getLength() * kGL * Math.cos(position)) + Math.pow(getLength(), 2) * kAL * accel;
    }

    public double calcSpringVoltage(double position) {
        double voltagePerTorque = Constants.ARM_KG / 344; //VOLTAGE TO HOLD ARM DIVIDED BY TORQUE TO HOLD IN IN-LBS
        double force = kSpring;
        double length = 23;
        double height = Units.metersToInches(Constants.ARMOFFSET.getZ()) - 5;
        double armAngle = getAngle().getRadians();
        double forceAngle = Math.atan2(height - (length * Math.sin(armAngle)), length * Math.cos(armAngle) - 6);
        double torque = force * Math.sin(forceAngle) * length * Math.cos(armAngle); 
        return -torque * voltagePerTorque;
    }

    public void setAngle(double angle) {
        TrapezoidProfile profile = new TrapezoidProfile(armConstraints, new TrapezoidProfile.State(angle, 0), pivotSetpoint);
        pivotSetpoint = profile.calculate(kDt);
        SmartDashboard.putNumber("setpointAngleVelocity", pivotSetpoint.velocity);
        SmartDashboard.putNumber("setpointAngleError", angle - getAngle().getRadians());
        SmartDashboard.putNumber("setpointAnglePosition", pivotSetpoint.position);
        setAngleV(pivotSetpoint.velocity);
    }

    //returns the required rotation to go to a setpoint in degrees (arm relative)
    public Rotation2d rotToPointAR(Translation3d target) {

        //defines a new translation3d with no y offset (side to side)
        Translation3d targetXZ = new Translation3d(target.getX(), 0.0, target.getZ());

        //angle to pivot to in degrees (rotating up is positive)
        Rotation2d angle = new Rotation2d(targetXZ.getX(), targetXZ.getZ());

        return angle;
    }

    //returns the required rotation to go to a setpoint in degrees (robot relative)
    public Rotation2d rotToPoint(Translation3d target) {
        return rotToPointAR(toArmRelative(target));
    }

    //returns the required length to go to a setpoint (arm relative)
    public double lengthToPointAR(Translation3d target) {
        //defines a new translation3d with no y offset (side to side)
        Translation2d targetXZ = new Translation2d(target.getX(), target.getZ());

        //calculates distance in the XZ plane (arm length)
        double length = targetXZ.getNorm();

        return length;
    }

    //returns the required length to go to a setpoint (robot relative)
    public double lengthToPoint(Translation3d target) {
        return lengthToPointAR(toArmRelative(target));
    }

    public boolean isLegalExtension(Translation3d target) {
        return this.getFurthestPoint(target).getX() < Units.inchesToMeters(45);
    }

    public boolean isLegalHeight(Translation3d target) {
        // left some inches since we shouldnt go this high ever (limit is 72" instead of legal 78")
        return this.getFurthestPoint(target).getY() < Units.inchesToMeters(72);
    }

    public boolean isFurther(Translation3d target) {
        return getTranslation().getNorm() < target.getNorm();
    }

    public SequentialCommandGroup goToSetpoint(Translation3d target) {
        //target = target.minus(new Translation3d(Constants.GRIPPERHOLDDISTANCE, new Rotation3d(0, RobotContainer.wrist.getAbsoluteAngle().getRadians(), 0)));
        
        var command = new SequentialCommandGroup();
        if(isFurther(target)) {
            command = new SequentialCommandGroup(new ArmControlPivot(rotToPoint(target).getRadians()), new ArmControlLength(lengthToPoint(target)));
        } else {
            command = new SequentialCommandGroup(new ArmControlLength(lengthToPoint(target)), new ArmControlPivot(rotToPoint(target).getRadians()));
        }
        return command;
    }

    public SequentialCommandGroup goToPivotLength(double pivot, double length) {
        var target = new Translation3d(length, new Rotation3d(0, pivot, 0));
        
        var command = new SequentialCommandGroup();
        if(isFurther(target)) {
            command = new SequentialCommandGroup(new ArmControlPivot(pivot), new ArmControlLength(length));
        } else {
            command = new SequentialCommandGroup(new ArmControlLength(length), new ArmControlPivot(pivot));
        }
        return command;
    }

    public Translation3d getTranslation() {
        return new Translation3d(getLength(), new Rotation3d(0, getAngle().getRadians(), 0));
    }

    private Translation2d getFurthestPoint(Translation3d target) {
        Translation2d targetXZ = new Translation2d(target.getX(), target.getZ());
        //Translation2d gripper = new Translation2d(Constants.GRIPPERLENGTH, targetXZ.getAngle().plus(RobotContainer.wrist.getRelativeAngle()));
        //return targetXZ.plus(gripper).plus(new Translation2d(Constants.ARMOFFSET.getX(), Constants.ARMOFFSET.getZ()));
        return targetXZ;
    }

    public void zeroArmLength() {
        extendEncoder.setPosition(0);
    }

}