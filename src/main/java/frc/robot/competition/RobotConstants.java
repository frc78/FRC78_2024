// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.util.Units;

/** This is the constants for the NEO */
class RobotConstants {
  public static final double WHEEL_WIDTH =
      Units.inchesToMeters(18.75); // Make sure this is from the wheel's center of rotation
  public static final double WHEEL_DIAMETER =
      Units.inchesToMeters(4); // TODO measure more precisely

  public static final double ROBOT_RADIUS = Math.hypot(WHEEL_WIDTH / 2.0, WHEEL_WIDTH / 2.0);

  public static final int PIGEON_ID = 0;

  public static final double MAX_SPEED = 4; // TODO
  public static final double MAX_ANGULAR_VELOCITY = 8; // TODO set temporarily, to look into later

  // WHEELS

  public static final double DRIVE_GEAR_RATIO = (6.75);
  public static final double DRIVE_ENC_TO_METERS = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
  public static final double DRIVE_ENC_VEL_TO_METERS_PER_SECOND =
      ((WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO) / 60;
  public static final boolean STEER_ENC_INVERTED = true;
  public static final boolean DRIVE_INVERTED = true;
  public static final boolean STEER_INVERTED = true;

  public static final double STEER_ENC_POS_TO_METERS =
      1; // factor of steer encoder to meters(conversion factor)
  public static final double STEER_ENC_VEL_TO_METERS =
      1 / 60; // factor of vel to meters

  public static final int DRIVE_CURRENT_LIMIT = 50; // amps
  public static final int STEER_CURRENT_LIMIT = 20; // amps

  public static final IdleMode DRIVE_IDLE = IdleMode.kBrake;
  public static final IdleMode STEER_IDLE = IdleMode.kCoast;

  public static final double STEER_ENC_PID_MIN = 0.0;
  public static final double STEER_ENC_PID_MAX = STEER_ENC_POS_TO_METERS; // TODO
}
