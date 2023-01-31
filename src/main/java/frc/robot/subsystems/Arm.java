package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Arm {
        static final Translation3d OFFSET = new Translation3d(0.0, 0.0, 0.0);


    //converts point from robot relative to arm relative
    public Translation3d toArmRelative(Translation3d robotRelative) {
        return robotRelative.minus(OFFSET);
    }

    //converts point from arm relative to robot relative
    public Translation3d toRobotRelative(Translation3d armRelative) {
        return armRelative.plus(OFFSET);
    }
    
        //extends the arm
    public void extendArm() {

    }

    //pivots the arm
    public void pivotArm() {

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
}