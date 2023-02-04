/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static final int XBOX_PORT = 0;
  public static final int SWITCHBOX_PORT = 1;
  public static final int JOYSTICK_PORT = 2;

  public static final int GRABBER_INTAKE = 30;
  public static final int GRABBER_PNEUMATIC_HUB = 31;
  public static final int GRABBER_COMPRESSOR = 32;
  public static final int GRABBER_SOLENOID_FORWARD = 33;
  public static final int GRABBER_SOLENOID_REVERSE = 34;
  public static final int GRABBER_SOLENOID = 35;

  public static final int FRONT_LEFT_PIVOT = 11;
  public static final int FRONT_RIGHT_PIVOT = 12;
  public static final int REAR_LEFT_PIVOT = 10;
  public static final int REAR_RIGHT_PIVOT = 13;

  public static final int FRONT_LEFT_DRIVE = 1;
  public static final int FRONT_RIGHT_DRIVE = 20;
  public static final int REAR_LEFT_DRIVE = 18;
  public static final int REAR_RIGHT_DRIVE = 19;

  public static final int ARM_PIVOT = 31;
  public static final int ARM_EXTEND = 32;

  public static final int FRONT_LEFT_DIGITAL_INPUT = 3;
  public static final int FRONT_RIGHT_DIGITAL_INPUT = 2;
  public static final int REAR_LEFT_DIGITAL_INPUT = 4;
  public static final int REAR_RIGHT_DIGITAL_INPUT = 5;
}