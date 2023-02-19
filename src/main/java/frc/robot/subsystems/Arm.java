package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Arm extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANCoder pivotCANCoder;

    private final CANSparkMax extendMotor;
    private final RelativeEncoder extendEncoder;

    private final ArmFeedforward armFeedFoward;
    private final PIDController armPID;

    private final SimpleMotorFeedforward extendFeedFoward;
    private final PIDController extendPID;
    private double kGL, kAL, kDt;

    TrapezoidProfile.Constraints armConstraints;

    static final Translation3d OFFSET = new Translation3d(0.0, 0.0, 0.0);

    public Arm (int pivotCANCoderChannel, int pivotMotorChannel, int extendMotorChannel, double pivotkS, double pivotkG, double pivotkV, double pivotkA, double pivotkP,
                double pivotkI, double pivotkD, double extendkS, double extendkV, double extendkA, double extendkP, double extendkI, double extendkD) {
        
                    pivotMotor = new CANSparkMax(pivotMotorChannel, MotorType.kBrushless);
        extendMotor = new CANSparkMax(extendMotorChannel, MotorType.kBrushless);

        pivotCANCoder = new CANCoder(pivotCANCoderChannel);
        extendEncoder = extendMotor.getEncoder();
        //DO THIS extendEncoder.setPositionConversionFactor();
        //DO THIS extendEncoder.setVelocityConversionFactor();
        armFeedFoward = new ArmFeedforward(pivotkS, pivotkG, pivotkV, pivotkA);
        armPID = new PIDController(pivotkP, pivotkI, pivotkD);
        
        extendFeedFoward = new SimpleMotorFeedforward(extendkS, extendkV, extendkA);
        extendPID = new PIDController(extendkP, extendkI, extendkD);

        armConstraints = new TrapezoidProfile.Constraints(1, 1);

        this.kGL = 0;
        this.kAL = 0;
        this.kDt = 0.02; //20 ms assumed for control loops
    }


    //converts point from robot relative to arm relative
    public Translation3d toArmRelative(Translation3d robotRelative) {
        return robotRelative.minus(OFFSET);
    }

    //converts point from arm relative to robot relative
    public Translation3d toRobotRelative(Translation3d armRelative) {
        return armRelative.plus(OFFSET);
    }
    
    //extends the arm
    public void setLength(double length) {
        TrapezoidProfile profile = new TrapezoidProfile(armConstraints, new TrapezoidProfile.State(length, 0), new TrapezoidProfile.State(getLength(), getLengthV()));
        TrapezoidProfile.State setpoint = profile.calculate(kDt);
        extendMotor.setVoltage(extendFeedFoward.calculate(setpoint.velocity) + extendPID.calculate(getLengthV(), setpoint.velocity));
    }

    //gets the arm extension
    public double getLength() {
        return extendEncoder.getPosition();
    }

    public void setLengthV(double vLength) {
        extendMotor.setVoltage(extendFeedFoward.calculate(vLength) + extendPID.calculate(getLengthV(), vLength));
    }

    public double getLengthV() {
        return extendEncoder.getVelocity();
    }

    //gets the arm angle
    public Rotation2d getAngle() {
        return new Rotation2d(Math.toRadians(pivotCANCoder.getAbsolutePosition()));
    }

    public void setAngleV(double vAngle) {
        pivotMotor.setVoltage(armFeedFoward.calculate(getAngle().getRadians() + vAngle * kDt, vAngle) + extendPID.calculate(getLengthV(), vAngle));
    }

    public double getAngleV() {
        return Math.toRadians(pivotCANCoder.getVelocity());
    }

    //takes in the position, vel, and accel setpoints, outputs the voltage for telescoping arm (rad, rad/s, rad/s^2)
    public double calcVoltagePivot(double position, double velocity, double accel) {
        return Constants.ARM_KS * Math.signum(velocity) + Constants.ARM_KV * velocity + (getLength() * kGL * Math.cos(position)) + Math.pow(getLength(), 2) * kAL * accel;
    }

    public void setAngle(double angle) {
        TrapezoidProfile profile = new TrapezoidProfile(armConstraints, new TrapezoidProfile.State(angle, 0), new TrapezoidProfile.State(getAngle().getRadians(), getAngleV()));
        TrapezoidProfile.State setpoint = profile.calculate(kDt);
        pivotMotor.setVoltage(armFeedFoward.calculate(getAngle().getRadians() + setpoint.velocity * kDt, setpoint.velocity) + armPID.calculate(getLengthV(), setpoint.velocity));
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
        Translation3d targetXZ = new Translation3d(target.getX(), 0.0, target.getZ());

        //calculates distance in the XZ plane (arm length)
        double length = targetXZ.getNorm();

        return length;
    }

    //returns the required length to go to a setpoint (robot relative)
    public double lengthToPoint(Translation3d target) {
        return lengthToPointAR(toArmRelative(target));
    }

    public void moveArmVelocity(double vx, double vz) {
        double length = getLength();
        Rotation2d angle = getAngle();
        double x = angle.getCos() * length;
        double z = angle.getSin() * length;
        //rate of change of length with respect to time
        double vLength = (((x * vx) + (z * vz)) / length);
        //rate of change of angle with respect to time
        double vAngle = (1 / (1 + Math.pow(x/z, 2))) * ((z * vx) - (x * vz)) / Math.pow(z, 2);
        setLengthV(vLength);
        setAngleV(vAngle);
    }

    public boolean isLegal(Translation3d target) {
        Translation2d targetXZ = new Translation2d(target.getX(), target.getY());
        Translation2d gripper = new Translation2d(Constants.GRIPPERLENGTH, targetXZ.getAngle().plus(RobotContainer.wrist.getRelativeAngle()));
        return targetXZ.plus(gripper).plus(new Translation2d(Constants.ARMOFFSET.getX(), Constants.ARMOFFSET.getZ())).getX() < Units.inchesToMeters(45) //45 inches to leave some tolerance
        && targetXZ.plus(gripper).plus(new Translation2d(Constants.ARMOFFSET.getX(), Constants.ARMOFFSET.getZ())).getY() < Units.inchesToMeters(72); //left some inches since we shouldnt go this high ever (limit is 72 in instead of legal 78)
    }
}