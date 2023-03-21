/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * constants
 */
public class Constants {
  public static final Transform3d CAMERA_1_POS = new Transform3d(
      new Translation3d(Units.inchesToMeters(14), Units.inchesToMeters(6.5), Units.inchesToMeters(9)),
      new Rotation3d(Math.toRadians(2), Math.toRadians(-15), 0));
  public static final Transform3d LIMELIGHT_POS = new Transform3d(
      new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
      new Rotation3d(Math.toRadians(0), Math.toRadians(0), 0));

  public static final Translation3d ARMOFFSET = new Translation3d(0.0, 0.0, Units.inchesToMeters(20));
  public static final double MIN_LENGTH = Units.inchesToMeters(24.0);
  public static final double GRIPPERLENGTH = Units.inchesToMeters(16);
  public static final double GRIPPERHOLDDISTANCE = Units.inchesToMeters(11);

  public static final Translation3d MID_POS = new Translation3d(Units.inchesToMeters(41.7), Units.inchesToMeters(0),
      Units.inchesToMeters(46));
  public static final Translation3d HIGH_POS = new Translation3d(Units.inchesToMeters(56.73), Units.inchesToMeters(0),
      Units.inchesToMeters(58));
  public static final Translation3d SHELF_POS = new Translation3d(Units.inchesToMeters(40), 0, Units.inchesToMeters(41.31));
  public static final Translation3d FLOOR_POS = new Translation3d(Units.inchesToMeters(30), 0, Units.inchesToMeters(10));
  public static final Translation3d SINGLE_POS = new Translation3d(Units.inchesToMeters(32), 0, Units.inchesToMeters(32));

  public static final double SENS = 0.5;

  public static final double DRIVE_VEL_ENCODER = 0.000637;
  public static final double DRIVE_POS_ENCODER = 0.038318;

  public static final double PIVOT_P = 0.01;
  public static final double PIVOT_I = 0.0;
  public static final double PIVOT_D = 0.0001;
  public static final double PIVOT_KS = 2.2064E-06;
  public static final double PIVOT_KV = 0.0;
  public static final double PIVOT_KA = 0.0;

  public static final double DRIVE_P = 0.01;
  public static final double DRIVE_I = 0;
  public static final double DRIVE_D = 0;
  public static final double DRIVE_KS = 0.11408;
  public static final double DRIVE_KV = 3.2717;
  public static final double DRIVE_KA = 0.17904;

  public static final double ARM_P = 8;
  public static final double ARM_I = 0.5;
  public static final double ARM_D = 0.01;
  public static final double ARM_KS = 0.26602;
  public static final double ARM_KG = 0.75656966792;
  public static final double ARM_KV = 4.5387;
  public static final double ARM_KA = 0.27838;

  public static final double ARM_START_ANGLE = Math.toRadians(35);

  public static final double EXTEND_P = 4;
  public static final double EXTEND_I = 0.6;
  public static final double EXTEND_D = 0.0;
  public static final double EXTEND_KS = 0.22841;
  public static final double EXTEND_KV = 5.3504;
  public static final double EXTEND_KA = 0.33319;
  public static final double EXTEND_KG = 0.88663;
  public static final double EXTEND_KSPRING = 0;

  public static final double WRIST_P = 2.5;
  public static final double WRIST_I = 0.4;
  public static final double WRIST_D = 0.16;
  public static final double WRIST_KS = 0.15463;
  public static final double WRIST_KG = 0.59328;
  public static final double WRIST_KV = 0.9972;
  public static final double WRIST_KA = 0.025145;

  public static final double WRIST_GEARING = 100;
  public static final double ARM_GEARING = 48;

  public static final double TARGET_P = 0.008;
  public static final double TARGET_I = 0.0;
  public static final double TARGET_D = 0.001;

  public static final int LED_LENGTH = 20;
}