package frc.robot.subsystems.vision;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;

/**
 * Wrapper class for getting and setting Limelight NetworkTable values.
 * 
 * https://gist.github.com/DanWaxman/3301ff38b13f776e70d8af9326ee8899
 *
 * @author https://github.com/DanWaxman
 */
public class Limelight {
  private static NetworkTableInstance table = null;

  /**
   * Light modes for Limelight.
   *
   * @author Dan Waxman
   */
  public static enum LightMode {
    eOn, eOff, eBlink
  }

  /**
   * Camera modes for Limelight.
   *
   * @author Dan Waxman
   */
  public static enum CameraMode {
    eVision, eDriver
  }

  /**
   * Gets whether a target is detected by the Limelight.
   *
   * @return true if a target is detected, false otherwise.
   */
  public static boolean isTarget() {
    return getValue("tv").getDouble(0) == 1;
  }

  /**
   * Horizontal offset from crosshair to target (-29.8 degrees to 29.8 degrees).
   *
   * @return tx as reported by the Limelight.
   */
  public static double getTx() {
    return getValue("tx").getDouble(0.00);
  }

  /**
   * Vertical offset from crosshair to target (-24.85 degrees to 24.85 degrees).
   *
   * @return ty as reported by the Limelight.
   */
  public static double getTy() {
    return getValue("ty").getDouble(0.00);
  }

  /**
   * Area that the detected target takes up in total camera FOV (0% to 100%).
   *
   * @return Area of target.
   */
  public static double getTa() {
    return getValue("ta").getDouble(0.00);
  }

  /**
   * Gets target skew or rotation (-90 degrees to 0 degrees).
   *
   * @return Target skew.
   */
  public static double getTs() {
    return getValue("ts").getDouble(0.00);
  }

  /**
   * Gets target latency (ms).
   *
   * @return Target latency.
   */
  public static double getTl() {
    return getValue("tl").getDouble(0.00);
  }

  /**
   * Sets LED mode of Limelight.
   *
   * @param mode Light mode for Limelight.
   */
  public static void setLedMode(LightMode mode) {
    getValue("ledMode").setNumber(mode.ordinal());
  }

  public static boolean isLedOn() {
    return (int) getValue("ledMode").getDouble(0) == (LightMode.eOn.ordinal());
  }

  /**
   * Sets camera mode for Limelight.
   *
   * @param mode Camera mode for Limelight.
   */
  public static void setCameraMode(CameraMode mode) {
    getValue("camMode").setNumber(mode.ordinal());
  }

  /**
   * Sets pipeline number (0-9 value).
   *
   * @param number Pipeline number (0-9).
   */
  public static void setPipeline(int number) {
    getValue("pipeline").setNumber(number);
  }

  /**
   * Helper method to get an entry from the Limelight NetworkTable.
   *
   * @param key Key for entry.
   * @return NetworkTableEntry of given entry.
   */
  private static NetworkTableEntry getValue(String key) {
    if (table == null) {
      table = NetworkTableInstance.getDefault();
    }

    return table.getTable("limelight").getEntry(key);
  }

  public static double getPitch() {
    return Math.toRadians(getTy() + Constants.LIMELIGHT_POS.getRotation().getY());
  }

  public static double getYaw() {
    return Math.toRadians(getTx() + Constants.LIMELIGHT_POS.getRotation().getZ());
  }
  
  public static double getLength(double height) {
    return (height - Constants.LIMELIGHT_POS.getY()) / Math.sin(getPitch());
  }

  public static double getDist(double height) {
    return (height - Constants.LIMELIGHT_POS.getY()) / Math.tan(getPitch());
  }

  public static Translation2d getTranslationRobotRelative(double height) {
    double dist = getDist(height);

    double x = dist * Math.cos(getYaw());
    double y = dist * Math.sin(getYaw());
    
    return new Translation2d(x, y).plus(Constants.LIMELIGHT_POS.getTranslation().toTranslation2d());
  }

  public static Pose2d getPoseTapeRelative(double height) {
    Rotation2d rotation = RobotContainer.swerveDrive.getPose().getRotation();
    Translation2d translation = getTranslationRobotRelative(height).rotateBy(rotation);
    return new Pose2d(translation, rotation);
  }

}