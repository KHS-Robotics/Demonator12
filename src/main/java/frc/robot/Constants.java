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
  public static final Transform3d CAMERA_1_POS = new Transform3d(new Translation3d(0.5, 0, 0.5),
      new Rotation3d(0, 0, 0));
  public static final double SENS = 0.5;

  public static final double ROBOT_HEIGHT = 0.5; // WAS 0.4
  public static final double TARGET_HEIGHT = 2.64;
  public static final double LIMELIGHT_HEIGHT = 0.87; // WAS 0.84
  public static final double LIMELIGHT_ANGLE = 40;

  public static final double FRONT_LEFT_P = 0.015;
  public static final double FRONT_LEFT_I = 0.0;
  public static final double FRONT_LEFT_D = 0.0;

  public static final double FRONT_RIGHT_P = 0.015;
  public static final double FRONT_RIGHT_D = 0.0;
  public static final double FRONT_RIGHT_I = 0.0;

  public static final double REAR_LEFT_P = 0.015;
  public static final double REAR_LEFT_I = 0.0;
  public static final double REAR_LEFT_D = 0.0;

  public static final double REAR_RIGHT_P = 0.015;
  public static final double REAR_RIGHT_I = 0.0;
  public static final double REAR_RIGHT_D = 0.0;

  public static final double DRIVE_VEL_ENCODER = 0.000637;
  public static final double DRIVE_POS_ENCODER = 0.038318;

  public static final double FRONT_RIGHT_DRIVE_P = 0.15;
  public static final double FRONT_RIGHT_DRIVE_I = 0.00055;
  public static final double FRONT_RIGHT_DRIVE_D = 3.0;
  public static final double FRONT_RIGHT_DRIVE_FF = 0.2857;

  public static final double FRONT_LEFT_DRIVE_P = 0.15;
  public static final double FRONT_LEFT_DRIVE_I = 0.00055;
  public static final double FRONT_LEFT_DRIVE_D = 3.0;
  public static final double FRONT_LEFT_DRIVE_FF = 0.2857;

  public static final double REAR_RIGHT_DRIVE_P = 0.15;
  public static final double REAR_RIGHT_DRIVE_I = 0.00055;
  public static final double REAR_RIGHT_DRIVE_D = 3.0;
  public static final double REAR_RIGHT_DRIVE_FF = 0.2857;

  public static final double REAR_LEFT_DRIVE_P = 0.15;
  public static final double REAR_LEFT_DRIVE_I = 0.00055;
  public static final double REAR_LEFT_DRIVE_D = 3.0;
  public static final double REAR_LEFT_DRIVE_FF = 0.2857;

  public static final double TARGET_P = 0.03;
  public static final double TARGET_I = 0.0035;
  public static final double TARGET_D = 0.0025;

  public static final Translation3d ARMOFFSET = new Translation3d(0.0, 0.0, 0.0);
}