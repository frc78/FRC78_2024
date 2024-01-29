// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import frc.robot.constants.Constants;

/** Utility class */
public class Util {
   /**
   * Adjusts the speeds of the given input depending on trigger input, with left
   * trigger decreasing speed and RT increasing.
   * 
   * Default speed = 1 - up adjust
   * Full left trigger = (1 - upAdjust) - down adjust
   * Full right trigger = 1
   * 
   * @param in
   * @return Adjusted speed
   */
  public static double triggerAdjust(double down, double up) {
    double triggers = (1 - Constants.UP_ADJUST) + (up * Constants.UP_ADJUST) - (Constants.TRIGGER_DEADBAND * Constants.DOWN_ADJUST);
    return triggers;
  }

   /**
   * Applies a deadband to the given joystick axis value
   * @param value
   * @param deadband
   * @return
   */
  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return (value > 0.0 ? value - deadband : value + deadband) / (1.0 - deadband);
    } else {
      return 0.0;
    }
  }

   /**
   * Processes the given joystick axis value, applying deadband and squaring it
   * @param value
   * @return
   */
  public static double modifyJoystick(double value) {
    // Deadband
    value = deadband(value, Constants.JOYSTICK_DEADBAND);
    // Square the axis
    // value = Math.copySign(value * value, value);
    return value;
  }
}