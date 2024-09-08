// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

/** This is the constants file for all of the common constants */
public final class Constants {
  public static final double JOYSTICK_DEADBAND = 0.15;
  public static final double TRIGGER_DEADBAND = 0.1;

  public static final double UP_ADJUST = 0.5;
  public static final double DOWN_ADJUST = 0.25;

  // TODO should revise and precisely measure these values, they are estimates
  public static final Translation2d BLUE_SPEAKER_POSE =
      new Translation2d(Meters.of(0), Meters.of(5.5));
  public static final Translation2d RED_SPEAKER_POSE =
      new Translation2d(Meters.of(16.5), Meters.of(5.5));
  public static final Measure<Distance> SPEAKER_HEIGHT = Meters.of(2.1); // is adjusted, not real
  public static final Translation2d BLUE_PLOP_POSE = new Translation2d(Meters.of(0), Meters.of(7));
  public static final Translation2d RED_PLOP_POSE =
      new Translation2d(Meters.of(16.5), Meters.of(7));

  public static final double GRAVITY = 9.807;
}
