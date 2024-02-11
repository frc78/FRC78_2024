// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import static frc.robot.subsystems.Shooter.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.classes.BaseDrive;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.FieldOrientedWithCardinal;
import frc.robot.commands.OrbitalTarget;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feedback;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterConfig;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.NeoModule;
import frc.robot.subsystems.chassis.PoseEstimator;
import frc.robot.subsystems.chassis.SwerveModule;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

class CompetitionRobotContainer {
  public final Chassis m_chassis;
  private final BaseDrive m_baseDrive;
  public final PoseEstimator m_poseEstimator;
  private final PhotonCamera m_ATCamera;
  private final Intake m_intake;
  private final Elevator m_Elevator;
  private final Shooter m_Shooter;
  private final Wrist m_Wrist;
  private final Feeder m_feeder;
  private final Feedback m_feedback;
  private final CommandXboxController m_driveController;
  private final CommandXboxController m_manipController;
  private final CommandXboxController m_testController;
  private final CommandXboxController sysIdController;
  private final SendableChooser<Command> autoChooser;

  CompetitionRobotContainer() {

    NeoModule frontLeft = new NeoModule(1, 2, RobotConstants.MODULE_CONFIG);
    ;
    NeoModule frontRight = new NeoModule(3, 4, RobotConstants.MODULE_CONFIG);
    ;
    NeoModule backLeft = new NeoModule(5, 6, RobotConstants.MODULE_CONFIG);
    ;
    NeoModule backRight = new NeoModule(7, 8, RobotConstants.MODULE_CONFIG);
    ;

    SwerveModule[] modules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};

    SwerveDriveKinematics swerveDriveKinematics = getSwerveDriveKinematics();

    m_ATCamera = new PhotonCamera(RobotConstants.AT_CAMERA_NAME);

    m_chassis = new Chassis(modules, swerveDriveKinematics);

    m_poseEstimator = new PoseEstimator(m_chassis, m_ATCamera, RobotConstants.PIGEON_ID);

    m_driveController = new CommandXboxController(0);
    m_manipController = new CommandXboxController(1);
    m_testController = new CommandXboxController(2);
    // Put on port 5 because we only want to use this during tests
    sysIdController = new CommandXboxController(5);

    m_baseDrive =
        new BaseDrive(
            m_driveController.getHID(), RobotConstants.MOTION_LIMITS, RobotConstants.RATE_LIMITS);

    PortForwarder.add(5800, "photonvision.local", 5800);

    m_chassis.setDefaultCommand(
        new FieldOrientedDrive(m_chassis, m_poseEstimator, m_baseDrive::calculateChassisSpeeds));

    m_intake =
        new Intake(
            RobotConstants.INTAKE_TOP_ID, RobotConstants.INTAKE_BOTTOM_ID,
            RobotConstants.INTAKE_SPEED_IN, RobotConstants.INTAKE_SPEED_OUT);

    m_Elevator = new Elevator();

    // TODO needs to be cleaned up, should probably be added to structs and created in constants
    ShooterConfig shooterConfig = new ShooterConfig();
    shooterConfig.FLYWHEEL_TOP_ID = RobotConstants.FLYWHEEL_TOP_ID;
    shooterConfig.FLYWHEEL_BOTTOM_ID = RobotConstants.FLYWHEEL_BOTTOM_ID;
    shooterConfig.FLYWHEEL_TOP_MIN = RobotConstants.FLYWHEEL_TOP_MIN;
    shooterConfig.FLYWHEEL_TOP_MAX = RobotConstants.FLYWHEEL_TOP_MAX;
    shooterConfig.FLYWHEEL_TOP_P = RobotConstants.FLYWHEEL_TOP_P;
    shooterConfig.FLYWHEEL_TOP_I = RobotConstants.FLYWHEEL_TOP_I;
    shooterConfig.FLYWHEEL_TOP_D = RobotConstants.FLYWHEEL_TOP_D;
    shooterConfig.FLYWHEEL_TOP_S = RobotConstants.FLYWHEEL_TOP_S;
    shooterConfig.FLYWHEEL_TOP_V = RobotConstants.FLYWHEEL_TOP_V;
    shooterConfig.FLYWHEEL_TOP_FF = RobotConstants.FLYWHEEL_TOP_FF;
    shooterConfig.FLYWHEEL_BOTTOM_MIN = RobotConstants.FLYWHEEL_BOTTOM_MIN;
    shooterConfig.FLYWHEEL_BOTTOM_MAX = RobotConstants.FLYWHEEL_BOTTOM_MAX;
    shooterConfig.FLYWHEEL_BOTTOM_P = RobotConstants.FLYWHEEL_BOTTOM_P;
    shooterConfig.FLYWHEEL_BOTTOM_I = RobotConstants.FLYWHEEL_BOTTOM_I;
    shooterConfig.FLYWHEEL_BOTTOM_D = RobotConstants.FLYWHEEL_BOTTOM_D;
    shooterConfig.FLYWHEEL_BOTTOM_S = RobotConstants.FLYWHEEL_BOTTOM_S;
    shooterConfig.FLYWHEEL_BOTTOM_V = RobotConstants.FLYWHEEL_BOTTOM_V;
    shooterConfig.FLYWHEEL_BOTTOM_FF = RobotConstants.FLYWHEEL_BOTTOM_FF;

    m_Shooter = new Shooter(shooterConfig);

    m_Wrist =
        new Wrist(
            RobotConstants.WRIST_ID, RobotConstants.WRIST_HIGH_LIM, RobotConstants.WRIST_LOW_LIM);

    m_feeder = new Feeder();

    m_feedback = new Feedback(1);

    NamedCommands.registerCommand(
        "StartShooter", m_Shooter.startShooter(RobotConstants.AUTO_SHOOT_SPEED));
    NamedCommands.registerCommand("RunIntake", m_intake.intakeCommand());
    NamedCommands.registerCommand("Score", m_feeder.fire());
    NamedCommands.registerCommand(
        "ScoreFromW2",
        m_Shooter
            .startShooter(RobotConstants.AUTO_SHOOT_SPEED)
            .alongWith(m_Wrist.setToTarget(125)));

    AutoBuilder.configureHolonomic(
        m_poseEstimator::getFusedPose, // Robot pose supplier
        m_poseEstimator::resetPose, // Method to reset odometry
        m_chassis::getRealChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        m_chassis::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        RobotConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
        () -> {
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

  private void configureBindings() {
    m_driveController
        .start()
        .onTrue(new InstantCommand(() -> m_poseEstimator.resetPose(new Pose2d())));
    m_driveController
        .rightBumper()
        .whileTrue(
            new OrbitalTarget(
                m_chassis,
                m_baseDrive::calculateChassisSpeeds,
                RobotConstants.TRANSLATION_PID,
                RobotConstants.ROTATION_PID,
                RobotConstants.MOTION_LIMITS,
                m_poseEstimator,
                () -> Constants.ORBIT_RADIUS,
                RobotConstants.ORBITAL_FF_CONSTANT));
    m_driveController
        .a()
        .or(m_driveController.b())
        .or(m_driveController.x())
        .or(m_driveController.y())
        .whileTrue(
            new FieldOrientedWithCardinal(
                m_chassis,
                m_poseEstimator,
                () -> {
                  double xCardinal =
                      (m_driveController.y().getAsBoolean() ? 1 : 0)
                          - (m_driveController.a().getAsBoolean() ? 1 : 0);
                  double yCardinal =
                      (m_driveController.b().getAsBoolean() ? 1 : 0)
                          - (m_driveController.x().getAsBoolean() ? 1 : 0);
                  double dir = -Math.atan2(yCardinal, xCardinal);
                  dir = dir < 0 ? dir + 2 * Math.PI : dir; // TODO check if needed

                  Logger.recordOutput("goalCardinal", dir);
                  return dir;
                },
                m_baseDrive::calculateChassisSpeeds,
                RobotConstants.ROTATION_PID,
                RobotConstants.ROTATION_CONSTRAINTS,
                RobotConstants.ROTATION_FF));

    // Zero the elevator when the robot leaves disabled mode and has not been zeroed
    RobotModeTriggers.disabled()
        .negate()
        .and(m_Elevator::hasNotBeenZeroed)
        .onTrue(m_Elevator.zeroElevator());

    m_manipController
        .leftTrigger(0.5)
        .whileTrue(m_Shooter.startShooter(500))
        .whileFalse(m_Shooter.stopCommand());

    m_testController.a().whileTrue(m_Wrist.setToTarget(90));

    m_manipController
        .y()
        .whileTrue(
            m_Wrist
                .setToTarget(110)
                .alongWith(m_Elevator.setToTarget(13.9))); // Sets to AMP // sets to STOW

    m_manipController.x().whileTrue(m_Wrist.setToTarget(125));

    m_manipController
        .rightBumper()
        .whileTrue(
            m_intake.intakeCommand().alongWith(m_feeder.runFeed()).until(m_feeder::isNoteQueued));

    m_manipController.leftBumper().whileTrue(m_feeder.reverseFeed());

    m_manipController.rightTrigger(0.5).whileTrue(m_feeder.fire());

    // The routine automatically stops the motors at the end of the command
    sysIdController.a().whileTrue(m_chassis.sysIdQuasistatic(Direction.kForward));
    sysIdController.b().whileTrue(m_chassis.sysIdDynamic(Direction.kForward));
    sysIdController.x().whileTrue(m_chassis.sysIdQuasistatic(Direction.kReverse));
    sysIdController.y().whileTrue(m_chassis.sysIdDynamic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
