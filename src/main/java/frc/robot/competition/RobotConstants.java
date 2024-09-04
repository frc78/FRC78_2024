// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import frc.robot.classes.ModuleConfig;
import frc.robot.classes.Structs.*;
import frc.robot.subsystems.Shooter.ShooterConfig;

/** This is the constants for the NEO */
class RobotConstants {
  public static final double WHEEL_WIDTH = 0.426; // Make sure this is from the wheel's center
  // of rotation
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

  public static final double ROBOT_RADIUS = Math.hypot(WHEEL_WIDTH / 2.0, WHEEL_WIDTH / 2.0);

  public static final int PIGEON_ID = 0;

  public static final String STERN_CAM_NAME = "SternCam";
  public static final String STARBOARD_CAM_NAME = "StarboardCam";
  public static final String PORT_CAM_NAME = "PortCam";
  public static final Transform3d STERN_CAM_POSE =
      new Transform3d(
          new Translation3d(-4.5, 0, 17.902).times(Units.inchesToMeters(1)),
          new Rotation3d(Math.PI, Math.toRadians(-30), Math.PI));

  public static final Transform3d STARBOARD_CAM_POSE =
      new Transform3d(
          new Translation3d(4.465, -10.205, 21.274).times(Units.inchesToMeters(1)),
          new Rotation3d(Math.PI, Math.toRadians(-25), Math.toRadians(-30)));

  public static final Transform3d PORT_CAM_POSE =
      new Transform3d(
          new Translation3d(4.465, 10.205, 21.274).times(Units.inchesToMeters(1)),
          new Rotation3d(0, Math.toRadians(-25), Math.toRadians(30)));

  public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1); // TODO
  public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(1, 1, 1.5);
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

  public static final MotionLimits MOTION_LIMITS = new MotionLimits(5.6, 3 /*TODO */, 12, 18);

  public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG =
      new HolonomicPathFollowerConfig(
          new PIDConstants(2, 0.0, 0.0), // Translation PID constants
          new PIDConstants(4, 0.0, 0.0), // Rotation PID constants
          RobotConstants.MOTION_LIMITS.maxSpeed, // Max module speed, in m/s
          RobotConstants.ROBOT_RADIUS, // Drive$ base radius in meters
          new ReplanningConfig(true, true) // Default path replanning config.
          );
  // TODO Since the above and below are both PID constants for moving the robot to
  // a target pose, perhaps we could use just one set of constants for both
  // Pathplanner and other drive commands?
  public static final PIDConstants TRANSLATION_PID = new PIDConstants(3.5, 0.0, 0.0);
  public static final PIDConstants ROTATION_PID = new PIDConstants(4.0, 0.0, 0.085);
  public static final Constraints ROTATION_CONSTRAINTS =
      new Constraints(MOTION_LIMITS.maxAngularSpeed, MOTION_LIMITS.maxAngularAcceleration);
  // TODO
  public static final FFConstants ROTATION_FF = new FFConstants(0.0, 0.0, 0.0);
  public static final double ORBITAL_FF_CONSTANT = 5;

  public static final RateLimits RATE_LIMITS = new RateLimits(11, 30);

  // WHEELS //
  public static final double DRIVE_GEAR_RATIO = (5.3571);
  public static final double STEER_GEAR_RATIO = 150 / 7;
  public static final double NEO_FREESPEED_RPS = 5676 / 60; // Free RPM of NEO to RPS
  public static final double DRIVE_WHEEL_FREESPEED =
      (NEO_FREESPEED_RPS * (WHEEL_DIAMETER * Math.PI)) / DRIVE_GEAR_RATIO; // Converted for wheel
  public static final double STEER_FREESPEED = (NEO_FREESPEED_RPS) / STEER_GEAR_RATIO;

  public static final double DRIVE_ENC_TO_METERS = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
  public static final double DRIVE_ENC_VEL_TO_METERS_PER_SECOND =
      ((WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO) / 60;
  public static final boolean STEER_ENC_INVERTED = true;
  public static final boolean DRIVE_INVERTED = true;
  public static final boolean STEER_INVERTED = true;

  public static final double STEER_ENC_POS_TO_METERS =
      1; // factor of steer encoder to meters(conversion factor)

  public static final double STEER_ENC_VEL_TO_METERS = 1.0 / 60.0; // factor of vel to meters

  public static final int DRIVE_CURRENT_LIMIT = 50; // amps
  public static final int STEER_CURRENT_LIMIT = 20; // amps

  public static final double NOMINAL_VOLTAGE = 12;

  public static final IdleMode DRIVE_IDLE = IdleMode.kBrake;
  public static final IdleMode STEER_IDLE = IdleMode.kBrake;

  public static final double STEER_ENC_PID_MIN = 0.0;
  public static final double STEER_ENC_PID_MAX = STEER_ENC_POS_TO_METERS; // TODO

  public static final FFConstants MODULE_FF[] = {
    new FFConstants(0.078918, 2.1152, 0.73299),
    new FFConstants(0.26707, 2.0848, 0.36198),
    new FFConstants(0.25259, 2.0883, 0.35247),
    new FFConstants(0.055245, 2.1739, 0.73292)
  };

  public static final ModuleConfig MODULE_CONFIG =
      new ModuleConfig(
          new ClosedLoopParameters(0.1, 0, 0, 0),
          new ClosedLoopParameters(20, 0, 1, 1 / STEER_FREESPEED),
          RobotConstants.DRIVE_ENC_TO_METERS,
          RobotConstants.DRIVE_ENC_VEL_TO_METERS_PER_SECOND,
          RobotConstants.STEER_ENC_POS_TO_METERS,
          RobotConstants.STEER_ENC_VEL_TO_METERS,
          RobotConstants.DRIVE_INVERTED,
          RobotConstants.STEER_INVERTED,
          RobotConstants.STEER_ENC_INVERTED,
          RobotConstants.STEER_ENC_PID_MIN,
          RobotConstants.STEER_ENC_PID_MAX,
          RobotConstants.DRIVE_CURRENT_LIMIT,
          RobotConstants.STEER_CURRENT_LIMIT,
          RobotConstants.NOMINAL_VOLTAGE,
          RobotConstants.DRIVE_IDLE,
          RobotConstants.STEER_IDLE);

  // INTAKE
  public static final int INTAKE_TOP_ID = 10;
  public static final int INTAKE_BOTTOM_ID = 9;

  public static final double INTAKE_SPEED_IN = 0.75;

  public static final double INTAKE_SPEED_OUT = -0.5;

  public static final double ELEVATOR_CLIMB_HEIGHT = 17;

  // FEED //
  public static final int FEED_ID = 16;

  // Time of flight sensor range of interest
  public static final double FEED_SENSOR_THRESHOLD = 300;

  public static final double FEED_INTAKE_SPEED = 0.15;
  public static final double FEED_OUTTAKE_SPEED = -1;
  public static final double FEED_FIRE_SPEED = 1;

  public static final double WRIST_SUB_AUTO_POS = 50;

  public static final ShooterConfig SHOOTER_CONFIG =
      new ShooterConfig(
          14,
          15,
          true,
          true,
          new Range(-1, 1),
          new Range(-1, 1),
          new PIDConstants(.15045, 0, 0),
          new PIDConstants(0.14965, 0, 0),
          new FFConstants(0.11599, 0.11275, 0.018537, 0.0),
          new FFConstants(0.12435, 0.1115, 0.018978, 0.0));

  // Wrist Constants
  public static final int WRIST_ID = 13;

  public static final Measure<Angle> WRIST_HIGH_LIM = Degrees.of(55);
  public static final Measure<Angle> WRIST_LOW_LIM = Degrees.of(0);

  public static final Measure<Angle> WRIST_PLOP_ANGLE =
      Degrees.of(1); // one degree (too lazy to do whole unit thing)

  // CANDLE //
  public static final int CANDLE_ID = 1;

  // TODO auto stuff, but what for and is it needed?
  public static final double AUTO_SHOOT_SPEED = 4000;
  public static final double CENTER_LINE_MARGIN = 0;

  // AUTO WRIST
  public static final Translation2d SHOOT_POINT = new Translation2d(0, 0.56); // TODO
  public static final double SHOOTER_RPM_TO_MPS =
      2 * (Math.PI * Units.inchesToMeters(2.65)) / 60; // Guess based on shooter wheel size
  //   public static final Range VELOCITY_RANGE =
  //       new Range(SHOOTER_RPM_TO_MPS * 5000, SHOOTER_RPM_TO_MPS * 5001);
  public static final double SHOOTER_VEL = 4000; // RPM
  public static final Range DISTANCE_RANGE = new Range(1.25, 5);
  public static final double HEIGHT_LENGTH_COEFF = 0.01;
  public static final double HIGH_DIST_COEFF = 2.7;
  public static final double STRAIGHT_DIST_COEFF = 1;
}
