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

/**
 * constants
 */
public class Constants {
  public static final Transform3d CAMERA_1_POS = new Transform3d(new Translation3d(0.5, 0, 0.5),
      new Rotation3d(0, 0, 0));
  public static final Translation3d ARMOFFSET = new Translation3d(0.0, 0.0, 0.0);
  public static final double GRIPPERLENGTH = 0;

  public static final double SENS = 0.5;

  public static final double FRONT_LEFT_P = 0.015;
  public static final double FRONT_LEFT_I = 0.0;
  public static final double FRONT_LEFT_D = 0.02;

  public static final double FRONT_RIGHT_P = 0.015;
  public static final double FRONT_RIGHT_I = 0.0;
  public static final double FRONT_RIGHT_D = 0.02;

  public static final double REAR_LEFT_P = 0.015;
  public static final double REAR_LEFT_I = 0.0;
  public static final double REAR_LEFT_D = 0.02;

  public static final double REAR_RIGHT_P = 0.015;
  public static final double REAR_RIGHT_I = 0.0;
  public static final double REAR_RIGHT_D = 0.02;

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
  public static final double FRONT_RIGHT_DRIVE_KV = 3.4035;
  public static final double FRONT_RIGHT_DRIVE_KA = 0.093048;

  public static final double FRONT_LEFT_DRIVE_P = 0.01;
  public static final double FRONT_LEFT_DRIVE_I = 0;
  public static final double FRONT_LEFT_DRIVE_D = 0;
  public static final double FRONT_LEFT_DRIVE_KS = 0.11408;
  public static final double FRONT_LEFT_DRIVE_KV = 3.4035;
  public static final double FRONT_LEFT_DRIVE_KA = 0.093048;

  public static final double REAR_RIGHT_DRIVE_P = 0.01;
  public static final double REAR_RIGHT_DRIVE_I = 0;
  public static final double REAR_RIGHT_DRIVE_D = 0;
  public static final double REAR_RIGHT_DRIVE_KS = 0.11408;
  public static final double REAR_RIGHT_DRIVE_KV = 3.4035;
  public static final double REAR_RIGHT_DRIVE_KA = 0.093048;

  public static final double REAR_LEFT_DRIVE_P = 0.01;
  public static final double REAR_LEFT_DRIVE_I = 0;
  public static final double REAR_LEFT_DRIVE_D = 0;
  public static final double REAR_LEFT_DRIVE_KS = 0.11408;
  public static final double REAR_LEFT_DRIVE_KV = 3.4035;
  public static final double REAR_LEFT_DRIVE_KA = 0.093048;

  public static final double ARM_P = 0;
  public static final double ARM_I = 0;
  public static final double ARM_D = 0;
  public static final double ARM_KS = 0;
  public static final double ARM_KG = 0;
  public static final double ARM_KV = 0;
  public static final double ARM_KA = 0;

  public static final double TARGET_P = 0.03;
  public static final double TARGET_I = 0.0035;
  public static final double TARGET_D = 0.0025;

  public static final int LED_LENGTH = 256;
}