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
      new Rotation3d(0, Math.toRadians(-15), 0));
  public static final Translation3d ARMOFFSET = new Translation3d(0.0, 0.0, Units.inchesToMeters(20));
  public static final double MIN_LENGTH = Units.inchesToMeters(24.0);
  public static final double GRIPPERLENGTH = Units.inchesToMeters(16);
  public static final double GRIPPERHOLDDISTANCE = Units.inchesToMeters(11);

  public static final Translation3d MID_POS = new Translation3d(Units.inchesToMeters(41.7), Units.inchesToMeters(0),
      Units.inchesToMeters(46));
  public static final Translation3d HIGH_POS = new Translation3d(Units.inchesToMeters(58.73), Units.inchesToMeters(0),
      Units.inchesToMeters(52));

  public static final double SENS = 0.5;

  public static final double FRONT_LEFT_P = 0.01;
  public static final double FRONT_LEFT_I = 0.0;
  public static final double FRONT_LEFT_D = 0.0001;

  public static final double FRONT_RIGHT_P = 0.01;
  public static final double FRONT_RIGHT_I = 0.0;
  public static final double FRONT_RIGHT_D = 0.0001;

  public static final double REAR_LEFT_P = 0.01;
  public static final double REAR_LEFT_I = 0.0;
  public static final double REAR_LEFT_D = 0.0001;

  public static final double REAR_RIGHT_P = 0.01;
  public static final double REAR_RIGHT_I = 0.0;
  public static final double REAR_RIGHT_D = 0.0001;

  public static final double FRONT_LEFT_KS = 2.2064E-06;
  public static final double FRONT_LEFT_KV = 0.0;
  public static final double FRONT_LEFT_KA = 0.0;

  public static final double FRONT_RIGHT_KS = 2.2064E-06;
  public static final double FRONT_RIGHT_KV = 0.0;
  public static final double FRONT_RIGHT_KA = 0.0;

  public static final double REAR_LEFT_KS = 2.2064E-06;
  public static final double REAR_LEFT_KV = 0.0;
  public static final double REAR_LEFT_KA = 0.0;

  public static final double REAR_RIGHT_KS = 2.2064E-06;
  public static final double REAR_RIGHT_KV = 0.0;
  public static final double REAR_RIGHT_KA = 0.0;

  public static final double DRIVE_VEL_ENCODER = 0.000637;
  public static final double DRIVE_POS_ENCODER = 0.038318;

  public static final double FRONT_RIGHT_DRIVE_P = 0.01;
  public static final double FRONT_RIGHT_DRIVE_I = 0;
  public static final double FRONT_RIGHT_DRIVE_D = 0;
  public static final double FRONT_RIGHT_DRIVE_KS = 0.11408;
  public static final double FRONT_RIGHT_DRIVE_KV = 3.2717;
  public static final double FRONT_RIGHT_DRIVE_KA = 0.17904;

  public static final double FRONT_LEFT_DRIVE_P = 0.01;
  public static final double FRONT_LEFT_DRIVE_I = 0;
  public static final double FRONT_LEFT_DRIVE_D = 0;
  public static final double FRONT_LEFT_DRIVE_KS = 0.11408;
  public static final double FRONT_LEFT_DRIVE_KV = 3.2717;
  public static final double FRONT_LEFT_DRIVE_KA = 0.17904;

  public static final double REAR_RIGHT_DRIVE_P = 0.01;
  public static final double REAR_RIGHT_DRIVE_I = 0;
  public static final double REAR_RIGHT_DRIVE_D = 0;
  public static final double REAR_RIGHT_DRIVE_KS = 0.11408;
  public static final double REAR_RIGHT_DRIVE_KV = 3.2717;
  public static final double REAR_RIGHT_DRIVE_KA = 0.17904;

  public static final double REAR_LEFT_DRIVE_P = 0.01;
  public static final double REAR_LEFT_DRIVE_I = 0;
  public static final double REAR_LEFT_DRIVE_D = 0;
  public static final double REAR_LEFT_DRIVE_KS = 0.11408;
  public static final double REAR_LEFT_DRIVE_KV = 3.2717;
  public static final double REAR_LEFT_DRIVE_KA = 0.17904;

  public static final double ARM_P = 0.5;
  public static final double ARM_I = 0.0;
  public static final double ARM_D = 0.01;
  public static final double ARM_KS = 0.26602;
  public static final double ARM_KG = 0.75656966792;
  public static final double ARM_KV = 4.5387;
  public static final double ARM_KA = 0.27838;

  public static final double ARM_START_ANGLE = Math.toRadians(35);

  public static final double EXTEND_P = 0.2;
  public static final double EXTEND_I = 0;
  public static final double EXTEND_D = 0.0;
  public static final double EXTEND_KS = 0.22841;
  public static final double EXTEND_KV = 5.3504;
  public static final double EXTEND_KA = 0.33319;
  public static final double EXTEND_KG = 0.88663;
  public static final double EXTEND_KSPRING = 0;

  public static final double WRIST_P = 1;
  public static final double WRIST_I = 0;
  public static final double WRIST_D = 0.005;
  public static final double WRIST_KS = 0.15463;
  public static final double WRIST_KG = 0.59328;
  public static final double WRIST_KV = 0.9972;
  public static final double WRIST_KA = 0.025145;

  public static final double WRIST_GEARING = 100;
  public static final double ARM_GEARING = 48;

  public static final double TARGET_P = 0.008;
  public static final double TARGET_I = 0.0;
  public static final double TARGET_D = 0.0001;

  public static final int LED_LENGTH = 88;
}