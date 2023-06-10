// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;
import frc.robot.Classes.ModuleConfig;

/** Add your docs here. */
public class RobotConstants {
    public static enum ROBOT_TYPE {WASP, ANTMAN};
    public static final ROBOT_TYPE ROBOT = ROBOT_TYPE.WASP;

    public static final double WHEEL_WIDTH = Units.inchesToMeters(18.75); //Make sure this is from the wheel's center of rotation
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4); //TODO

    public static final ModuleConfig MOD_0_CONFIG = new ModuleConfig(1, 2, 9, 0.0);
    public static final ModuleConfig MOD_1_CONFIG = new ModuleConfig(3, 4, 10, 0.0);
    public static final ModuleConfig MOD_2_CONFIG = new ModuleConfig(5, 6, 11, 0.0);
    public static final ModuleConfig MOD_3_CONFIG = new ModuleConfig(7, 8, 12, 0.0);
}
