package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final CANSparkMax pivotMotor;
    private final CANSparkMax extendMotor;

    public Arm (int pivotMotorChannel, int extendMotorChannel) {
        pivotMotor = new CANSparkMax(pivotMotorChannel, MotorType.kBrushless);
        extendMotor = new CANSparkMax(extendMotorChannel, MotorType.kBrushless);
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

    }

    //gets the arm extension
    public double getLength() {
        return 0.0;
    }

    public void setLengthV(double vLength) {

    }

    //pivots the arm
    public void setAngle(Rotation2d Angle) {

    }

    //gets the arm angle
    public Rotation2d getAngle() {
        return new Rotation2d();
    }

    public void setAngleV(double vAngle) {

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
}