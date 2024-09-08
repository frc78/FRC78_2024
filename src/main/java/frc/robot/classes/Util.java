// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.classes.Structs.Range;
import frc.robot.constants.Constants;

/** Utility class */
public class Util {
  /**
   * Adjusts the speeds of the given input depending on trigger input, with left trigger decreasing
   * speed and RT increasing.
   *
   * <p>Default speed = 1 - up adjust Full left trigger = (1 - upAdjust) - down adjust Full right
   * trigger = 1
   *
   * @param in
   * @return Adjusted speed
   */
  public static double triggerAdjust(double down, double up) {
    double triggers =
        (1 - Constants.UP_ADJUST) + (up * Constants.UP_ADJUST) - (down * Constants.DOWN_ADJUST);
    return triggers;
  }

  public static double modifyTrigger(double value) {
    return MathUtil.applyDeadband(value, Constants.TRIGGER_DEADBAND);
  }

  /**
   * Processes the given joystick axis value, applying deadband and squaring it
   *
   * @param value
   * @return
   */
  public static double modifyJoystick(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, Constants.JOYSTICK_DEADBAND);
    // value = Math.pow(value, 1.5) * Math.signum(value);
    // Square the axis
    // value = Math.copySign(value * value, value);
    return value;
  }

  public static Translation2d normalize(Translation2d vector) {
    return new Translation2d(vector.getX() / vector.getNorm(), vector.getY() / vector.getNorm());
  }

  /*Returns right-hand perpendicular vector */
  public static Translation2d perpendicular(Translation2d vector) {
    return new Translation2d(vector.getY(), -vector.getX());
  }

  public static double clamp(double x, Range range) {
    return Math.max(range.min, Math.min(range.max, x));
  }

  public static double clamp(double x, double min, double max) {
    return Math.max(min, Math.min(max, x));
  }

  public static double lerp(double x, Range range) {
    return range.min + (range.max - range.min) * x;
  }
}
