package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax extendMotor;
    private final ArmFeedforward armFeedFoward;
    private final PIDController armPID;
    private double kS, kV, kGL, kAL;

    static final Translation3d OFFSET = new Translation3d(0.0, 0.0, 0.0);

    public Arm (int pivotMotorChannel, int extendMotorChannel, double kS, double kG, double kV, double kA, double kP, double kI, double kD) {
        pivotMotor = new CANSparkMax(pivotMotorChannel, MotorType.kBrushless);
        extendMotor = new CANSparkMax(extendMotorChannel, MotorType.kBrushless);
        armFeedFoward = new ArmFeedforward(kS, kG, kV, kA);
        armPID = new PIDController(kP, kI, kD);
        this.kS = kS;
        this.kV = kV;
        this.kGL = 0;
        this.kAL = 0;
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

    }

    //gets the arm extension
    public double getLength() {
        return 0.0;
    }

    public void setLengthV(double vLength) {

    }

    //gets the arm angle
    public Rotation2d getAngle() {
        return new Rotation2d();
    }

    public void setAngleV(double vAngle) {

    }
    //takes in the position, vel, and accel setpoints, outputs the voltage for telescoping arm (rad, rad/s, rad/s^2)
    public double calcVoltage(double position, double velocity, double accel) {
        return kS * Math.signum(velocity) + kV * velocity + (getLength() * kGL * Math.cos(position)) + Math.pow(getLength(), 2) * kAL * accel;
    }

    public void setAngle(double position, double velocity, double accel) {
        pivotMotor.setVoltage(armFeedFoward.calculate(position, velocity, accel) + armPID.calculate(getAngle().getRadians(), position));
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
        Translation2d gripper = new Translation2d(Constants.GRIPPERLENGTH, targetXZ.getAngle().plus(new Rotation2d() /*GRIPPER ANGLE GOES HERE */));
        return targetXZ.plus(gripper).getX() < 45; //45 inches to leave some tolerance
    }
}