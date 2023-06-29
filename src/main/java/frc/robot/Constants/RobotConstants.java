// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;
import frc.robot.Classes.ModuleConfig;
import frc.robot.Constants.Constants.ROBOT_TYPE;

//Switch between the WASP and NEO by commenting out the other one

/** This is the constants for the WASP */
public class RobotConstants {
    public static final ROBOT_TYPE ROBOT = ROBOT_TYPE.WASP;

    public static final double WHEEL_WIDTH = Units.inchesToMeters(18.75); //Make sure this is from the wheel's center of rotation
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4); //TODO
    
    public static final ModuleConfig[] MOD_CONFIGS = {
        new ModuleConfig(1, 2, 9, 0.0),
        new ModuleConfig(3, 4, 10, 0.0),
        new ModuleConfig(5, 6, 11, 0.0),
        new ModuleConfig(7, 8, 12, 0.0)
    };
}

/** This is the constants for the NEO */
// public class RobotConstants {
//     public static final ROBOT_TYPE ROBOT = ROBOT_TYPE.NEO;

//     public static final double WHEEL_WIDTH = Units.inchesToMeters(18.75); //Make sure this is from the wheel's center of rotation
//     public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4); //TODO
    
//     public static final ModuleConfig[] MOD_CONFIGS = {
//         new ModuleConfig(1, 2, 9, 0.0),
//         new ModuleConfig(3, 4, 10, 0.0),
//         new ModuleConfig(5, 6, 11, 0.0),
//         new ModuleConfig(7, 8, 12, 0.0)
//     };
// }