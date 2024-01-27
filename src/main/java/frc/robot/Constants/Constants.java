// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** This is the constants file for all of the common constants */
public final class Constants {
    public static enum ROBOT_TYPE {NEO};
    
    public static final double JOYSTICK_DEADBAND = 0.1;
    public static final double TRIGGER_DEADBAND = 0.1;

    public static final double UP_ADJUST = 0.5;
    public static final double DOWN_ADJUST = 0.25;

    public static final Translation2d BLUE_ORBIT_POSE = new Translation2d(0.5, 5.5);
    public static final Translation2d RED_ORBIT_POSE = new Translation2d(16, 5.5);
    public static final double ORBIT_RADIUS = 2.5;
    public static final double ORBIT_RADIUS_MARGIN = 1.0;

    // +X is forward, +Y is left
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0, RobotConstants.WHEEL_WIDTH / 2.0), //front left
            new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0, -RobotConstants.WHEEL_WIDTH / 2.0), //front right 
            new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0, RobotConstants.WHEEL_WIDTH / 2.0), //back left
            new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0, -RobotConstants.WHEEL_WIDTH / 2.0)); //back right

    public static final double ROBOT_RADIUS = (RobotConstants.WHEEL_WIDTH / 2) * Math.sqrt(2); // Only works if drivebase is square

        // public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        //     new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0, RobotConstants.WHEEL_WIDTH / 2.0), //front left
        //     new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0, -RobotConstants.WHEEL_WIDTH / 2.0), //front right 
        //     new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0, RobotConstants.WHEEL_WIDTH / 2.0), //back left
        //     new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0, -RobotConstants.WHEEL_WIDTH / 2.0)); //back right 
}