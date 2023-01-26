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

  public static final int FRONT_LEFT_PIVOT = 11;
  public static final int FRONT_RIGHT_PIVOT = 12;
  public static final int REAR_LEFT_PIVOT = 10;
  public static final int REAR_RIGHT_PIVOT = 13;

  public static final int FRONT_LEFT_DRIVE = 1;
  public static final int FRONT_RIGHT_DRIVE = 20;
  public static final int REAR_LEFT_DRIVE = 18;
  public static final int REAR_RIGHT_DRIVE = 19;

  public static final int FRONT_LEFT_DIGITAL_INPUT = 3;
  public static final int FRONT_RIGHT_DIGITAL_INPUT = 2;
  public static final int REAR_LEFT_DIGITAL_INPUT = 4;
  public static final int REAR_RIGHT_DIGITAL_INPUT = 5;
  public static final int INDEXER_BEAM_BREAK = 6;
  public static final int SHOOTER_BEAM_BREAK = 7;

  public static final int INTAKE_DRIVE = 3;
  public static final int INTAKE_POSITION = 2;

  public static final int INDEXER_LEFT = 8;
  public static final int INDEXER_RIGHT = 9;
  public static final int INDEXER_FEEDER = 5;

  public static final int SHOOTER_LEADER = 7;
  public static final int SHOOTER_FOLLOWER = 6;

  public static final int HOOD_SERVO_1 = 8;
  public static final int HOOD_SERVO_2 = 9;

  public static final int ELEVATOR_LEADER = 15;
  public static final int ELEVATOR_FOLLOWER1 = 16;
  public static final int ELEVATOR_FOLLOWER2 = 17;
  public static final int PIVOT_MOTOR = 14;
}