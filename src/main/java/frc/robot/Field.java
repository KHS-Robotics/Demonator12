/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * constants
 */
public class Field {
  // offsets of nodes (from apriltag)
  public static final Translation3d[] NODEOFFSETS = {
      new Translation3d(Units.inchesToMeters(39.73), Units.inchesToMeters(-22.0), Units.inchesToMeters(46.0)),
      new Translation3d(Units.inchesToMeters(42.82), 0.0, Units.inchesToMeters(35.50)),
      new Translation3d(Units.inchesToMeters(39.73), Units.inchesToMeters(22.0), Units.inchesToMeters(46.0)),
      new Translation3d(Units.inchesToMeters(22.7), Units.inchesToMeters(-22.0), Units.inchesToMeters(34.00)),
      new Translation3d(Units.inchesToMeters(22.935), 0.0, Units.inchesToMeters(23.53)),
      new Translation3d(Units.inchesToMeters(22.7), Units.inchesToMeters(22.0), Units.inchesToMeters(34.00)),
      new Translation3d(Units.inchesToMeters(-7.14), Units.inchesToMeters(-22.0), 0.0),
      new Translation3d(Units.inchesToMeters(-7.14), 0.0, 0.0),
      new Translation3d(Units.inchesToMeters(-7.14), Units.inchesToMeters(22.0), 0.0)
  };

  public static enum GridScoringPosition {
    TOP_LEFT, TOP_MID, TOP_RIGHT,
    MID_LEFT, MID_MID, MID_RIGHT,
    BOTTOM_LEFT, BOTTOM_MID, BOTTOM_RIGHT
  }

  public static AprilTagFieldLayout fieldLayout;
  static {
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public static final double HIGH_TAPE_HEIGHT = Units.inchesToMeters(43.81); //tape goes from 41.81 in to 45.81
  public static final double MID_TAPE_HEIGHT = Units.inchesToMeters(24); //tape from 22 to 26 in

  public static final double DIST_FROM_NODE_X_METERS = Units.inchesToMeters(40.28);
  public static final double PLACEMENTX = fieldLayout.getTagPose(8).get().getX() + DIST_FROM_NODE_X_METERS;

  // flips coordinates to be correctly aligned with alliance
  public static Translation3d flip(Translation3d translation) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return new Translation3d(Units.inchesToMeters(651.25) - translation.getX(),
          Units.inchesToMeters(315.5) - translation.getY(),
          translation.getZ());
    } else {
      return translation;
    }

  }

  //flips pose to be correctly aligned with alliance
  public static Pose2d flipPose(Pose2d pose) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return new Pose2d(Units.inchesToMeters(651.25) - pose.getX(),
      Units.inchesToMeters(315.5) - pose.getY(), 
      pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    else {
      return pose;
    }
  }

  // takes apriltag num from 1-8 and node num from 0-8
  public static Translation3d getNodeCoordinatesFieldRelative(int apriltag, int node) {
    return flip(fieldLayout.getTagPose(apriltag).get().getTranslation()).plus(NODEOFFSETS[node]);
  }

  public static double[] getNodeYArray() {
    double[] array = {flip(getNodeCoordinatesFieldRelative(8, 0)).getY(),
      flip(getNodeCoordinatesFieldRelative(8, 1)).getY(),
      flip(getNodeCoordinatesFieldRelative(8, 2)).getY(),
      flip(getNodeCoordinatesFieldRelative(7, 0)).getY(),
      flip(getNodeCoordinatesFieldRelative(7, 1)).getY(),
      flip(getNodeCoordinatesFieldRelative(7, 2)).getY(),
      flip(getNodeCoordinatesFieldRelative(6, 0)).getY(),
      flip(getNodeCoordinatesFieldRelative(6, 1)).getY(),
      flip(getNodeCoordinatesFieldRelative(6, 2)).getY()};
    return array;
  }

  public static enum GridPosition {
    LEFT(-1), MID(0), RIGHT(1);

    public final int value;

    private GridPosition(int value) {
      this.value = value;
    }

  }

  public static int aprilTagFromInput(GridPosition position) {
    var isBlueAlliance = DriverStation.getAlliance() == Alliance.Blue;
    switch (position) {
      case RIGHT:
        return isBlueAlliance ? 8 : 3;
      case MID:
        return isBlueAlliance ? 7 : 2;
      case LEFT:
        return isBlueAlliance ? 6 : 1;
      default:
        DriverStation.reportError("Invalid grid apriltag", false);
        return -1;
    }
  }
}