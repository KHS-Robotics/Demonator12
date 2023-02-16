/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * constants
 */
public class Field {
    //offsets of nodes (from apriltag)
    public static final Translation3d[] NODEOFFSETS = {
            new Translation3d(Units.inchesToMeters(39.73), Units.inchesToMeters(-22.0), Units.inchesToMeters(46.0)),
            new Translation3d(Units.inchesToMeters(42.82), 0.0, Units.inchesToMeters(35.50)),
            new Translation3d(Units.inchesToMeters(39.73), Units.inchesToMeters(22.0), Units.inchesToMeters(46.0)),
            new Translation3d(Units.inchesToMeters(22.7), Units.inchesToMeters(-22.0), Units.inchesToMeters(34.00)),
            new Translation3d(Units.inchesToMeters(22.935), 0.0, Units.inchesToMeters(23.53)),
            new Translation3d(Units.inchesToMeters(22.7), Units.inchesToMeters(22.0), Units.inchesToMeters(34.00)),
            new Translation3d(Units.inchesToMeters(-7.14), Units.inchesToMeters(-22.0), 0.0),
            new Translation3d(Units.inchesToMeters(-7.14), 0.0, 0.0),
            new Translation3d(Units.inchesToMeters(-7.14), Units.inchesToMeters(22.0), 0.0) };

    //positions of apriltags (blue alliance)
    public static final Translation3d[] APRILTAGS = {
            new Translation3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22)),
            new Translation3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22)),
            new Translation3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(174.19),
                    Units.inchesToMeters(18.22)),
            new Translation3d(
                    Units.inchesToMeters(636.96),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38)),
            new Translation3d(
                    Units.inchesToMeters(14.25),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38)),
            new Translation3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(174.19),
                    Units.inchesToMeters(18.22)),
            new Translation3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22)),
            new Translation3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22))
    };

    // flips coordinates to be correctly aligned with alliance
    public static Translation3d flip(Translation3d translation) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return new Translation3d(Units.inchesToMeters(651.25) - translation.getX(), Units.inchesToMeters(315.5) - translation.getY(),
                    translation.getZ());
        } else {
            return translation;
        }

    }

    // takes apriltag num from 1-8 and node num from 0-8
    public static Translation3d getNodeCoordinatesFieldRelative(int apriltag, int node) {
        return flip(APRILTAGS[apriltag - 1]).plus(NODEOFFSETS[node]);
    }
}