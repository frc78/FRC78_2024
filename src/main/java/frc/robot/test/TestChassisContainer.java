// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.classes.ModuleConfig;
import frc.robot.commands.*;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.NeoModule;
import frc.robot.subsystems.chassis.SwerveModule;

class TestChassisContainer {
  private final Chassis m_chassis;
  private final XboxController m_driveController;
  private final SendableChooser<Command> autoChooser;

  TestChassisContainer() {

    NeoModule frontLeft = makeSwerveModule(1, 2);
    NeoModule frontRight = makeSwerveModule(3, 4);
    NeoModule backLeft = makeSwerveModule(5, 6);
    NeoModule backRight = makeSwerveModule(7, 8);

    SwerveModule[] modules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};

    SwerveDriveKinematics swerveDriveKinematics = getSwerveDriveKinematics();

    m_chassis = new Chassis(modules, swerveDriveKinematics, RobotConstants.PIGEON_ID);

    m_driveController = new XboxController(0);

    m_chassis.setDefaultCommand(
        new Drive(
            m_chassis,
            m_driveController::getLeftY,
            m_driveController::getLeftX,
            m_driveController::getRightX,
            m_driveController::getLeftTriggerAxis,
            m_driveController::getRightTriggerAxis,
            m_driveController::getYButton,
            m_driveController::getBButton,
            m_driveController::getAButton,
            m_driveController::getXButton,
            RobotConstants.MAX_SPEED,
            RobotConstants.MAX_ANGULAR_VELOCITY));

    AutoBuilder.configureHolonomic(
        m_chassis::getFusedPose, // Robot pose supplier
        m_chassis
            ::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        m_chassis::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_chassis::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
            RobotConstants.MAX_SPEED, // Max module speed, in m/s
            RobotConstants
                .ROBOT_RADIUS, // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        m_chassis // Reference to this subsystem to set requirements
        );

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoMode", autoChooser);

    configureBindings();
  }

  private static SwerveDriveKinematics getSwerveDriveKinematics() {
    Translation2d frontLeftLocation =
        new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0, RobotConstants.WHEEL_WIDTH / 2.0);
    Translation2d frontRightLocation =
        new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0, -RobotConstants.WHEEL_WIDTH / 2.0);
    Translation2d backLeftLocation =
        new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0, RobotConstants.WHEEL_WIDTH / 2.0);
    Translation2d backRightLocation =
        new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0, -RobotConstants.WHEEL_WIDTH / 2.0);

    return new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  }

  private NeoModule makeSwerveModule(int driveId, int steerId) {
    ModuleConfig.ClosedLoopParameters driveClosedLoopParams =
        new ModuleConfig.ClosedLoopParameters(0.1, 0, 0, 1);
    ModuleConfig.ClosedLoopParameters steerClosedLoopParams =
        new ModuleConfig.ClosedLoopParameters(0.7, 0, 0, 0);
    return new NeoModule(
        new ModuleConfig(
            driveId,
            steerId,
            driveClosedLoopParams,
            steerClosedLoopParams,
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
            RobotConstants.DRIVE_IDLE,
            RobotConstants.STEER_IDLE));
  }

  private void configureBindings() {
    new Trigger(m_driveController::getStartButton)
        .onTrue(new InstantCommand(() -> m_chassis.resetPose(new Pose2d())));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
