// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.classes.BaseDrive;
import frc.robot.classes.TunerConstants;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.VarFeedPrime;
import frc.robot.commands.VarShootPrime;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feedback;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.chassis.Vision;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

class CompetitionRobotContainer {

  public final CommandSwerveDrivetrain m_chassis;
  private final BaseDrive m_baseDrive;
  public final Vision m_vision;
  private final Intake m_intake;
  private final Elevator m_Elevator;
  public final Shooter m_Shooter;
  private final Wrist m_Wrist;
  private final Feeder m_feeder;
  final Feedback m_feedback;
  private final CommandXboxController m_driveController;
  private final CommandXboxController m_manipController;
  private final CommandXboxController m_testController;
  private final CommandXboxController sysIdController;
  private final SendableChooser<Command> autoChooser;
  private final Command AmpSetUp;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private final SwerveDriveBrake brakeMode = new SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle();

  CompetitionRobotContainer() {

    fieldCentricFacingAngle.ForwardReference = SwerveRequest.ForwardReference.RedAlliance;
    fieldCentricFacingAngle.HeadingController.setP(4);
    fieldCentricFacingAngle.HeadingController.enableContinuousInput(0, Units.degreesToRadians(360));
    fieldCentricFacingAngle.HeadingController.setTolerance(Units.degreesToRadians(2.0));

    m_chassis =
        new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

    m_vision = new Vision(m_chassis);

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
        m_chassis.applyRequest(
            () -> {
              ChassisSpeeds speeds = m_baseDrive.calculateChassisSpeeds();
              return drive
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond);
            }));

    m_intake =
        new Intake(
            RobotConstants.INTAKE_TOP_ID, RobotConstants.INTAKE_BOTTOM_ID,
            RobotConstants.INTAKE_SPEED_IN, RobotConstants.INTAKE_SPEED_OUT);

    m_Elevator = new Elevator();

    m_Shooter = new Shooter(RobotConstants.SHOOTER_CONFIG);

    m_Wrist =
        new Wrist(
            RobotConstants.WRIST_ID, RobotConstants.WRIST_HIGH_LIM, RobotConstants.WRIST_LOW_LIM);

    m_feeder = new Feeder(RobotConstants.FEED_ID);

    m_feedback = new Feedback(RobotConstants.CANDLE_ID);

    AmpSetUp =
        (m_Wrist
            .setToTargetCmd(Degrees.of(19))
            .alongWith(m_Elevator.setToTarget(13.9))
            .withName("Amp Set Up"));

    NamedCommands.registerCommand("Intake", pickUpNote());
    NamedCommands.registerCommand("StopShooter", m_Shooter.setSpeedCmd(0));

    NamedCommands.registerCommand(
        "StartShooter", m_Shooter.setSpeedCmd(RobotConstants.AUTO_SHOOT_SPEED));
    NamedCommands.registerCommand(
        "Score", Commands.waitSeconds(0.5).andThen(m_feeder.shoot()).withName("Score"));

    NamedCommands.registerCommand("AmpSetUp", AmpSetUp);
    NamedCommands.registerCommand(
        "scoreInAmp", m_feeder.outtake().withTimeout(2).withName("scoreInAmp"));
    NamedCommands.registerCommand("stow", m_Wrist.stow());
    NamedCommands.registerCommand(
        "Target",
        m_chassis
            .applyRequest(
                () -> {
                  Translation2d target =
                      DriverStation.getAlliance().orElse(Blue) == Red
                          ? Constants.RED_SPEAKER_POSE
                          : Constants.BLUE_SPEAKER_POSE;
                  Rotation2d angle =
                      target
                          .minus(m_chassis.getState().Pose.getTranslation())
                          .getAngle()
                          .plus(Rotation2d.fromRadians(Math.PI));
                  Logger.recordOutput("Aiming angle", angle);
                  return fieldCentricFacingAngle.withTargetDirection(angle);
                })
            .until(() -> fieldCentricFacingAngle.HeadingController.atSetpoint())
            .withTimeout(1)
            .withName("Target"));
    NamedCommands.registerCommand("StopShooter", m_Shooter.setSpeedCmd(0));
    NamedCommands.registerCommand(
        "DriveToNote",
        pickUpNote()
            .deadlineWith(new AlignToNote(m_chassis, () -> new ChassisSpeeds(1.5, 0, 0)))
            .withTimeout(2)
            .withName("Drive to Note"));
    NamedCommands.registerCommand(
        "DriveToNote",
        pickUpNote()
            .deadlineWith(new AlignToNote(m_chassis, () -> new ChassisSpeeds(1.5, 0, 0)))
            .until(
                () -> {
                  if (DriverStation.getAlliance().orElse(Blue) == Blue) {
                    return m_chassis.getState().Pose.getX()
                        > 8.25 + RobotConstants.CENTER_LINE_MARGIN;
                  } else {
                    return m_chassis.getState().Pose.getX()
                        < 8.25 - RobotConstants.CENTER_LINE_MARGIN;
                  }
                })
            .withName("Drive to Note"));
    NamedCommands.registerCommand("Stow", m_Wrist.stow());
    NamedCommands.registerCommand(
        "VariableShoot",
        new VarShootPrime(
            m_Wrist,
            m_Elevator,
            m_chassis,
            RobotConstants.SHOOT_POINT,
            () -> m_Shooter.getVelocity() * 60,
            RobotConstants.DISTANCE_RANGE,
            RobotConstants.HEIGHT_LENGTH_COEFF,
            RobotConstants.SHOOTER_RPM_TO_MPS,
            RobotConstants.WRIST_HIGH_LIM));

    // Need to add and then to stop the feed and shooter

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoMode", autoChooser);

    configureBindings();
  }

  Command shortRumble(XboxController controller, RumbleType rumbleType) {
    return Commands.startEnd(
            () -> controller.setRumble(rumbleType, 1),
            () -> controller.setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(0.5)
        .withName("Rumble");
  }

  private void configureBindings() {
    new Trigger(m_feeder::isNoteQueued)
        .whileTrue(m_feedback.noteInCartridge())
        .and(RobotModeTriggers.teleop())
        .onTrue(shortRumble(m_driveController.getHID(), RumbleType.kRightRumble))
        .onTrue(shortRumble(m_manipController.getHID(), RumbleType.kRightRumble))
        .onFalse(shortRumble(m_driveController.getHID(), RumbleType.kBothRumble));

    // Rumble controllers when target is detected and we don't have a note
    NetworkTableEntry limelightTargetDetected =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    new Trigger(() -> limelightTargetDetected.getDouble(0.0) == 1.0)
        .and(RobotModeTriggers.teleop())
        .and(() -> !m_intake.hasNote())
        .onTrue(shortRumble(m_driveController.getHID(), RumbleType.kLeftRumble))
        .onTrue(shortRumble(m_manipController.getHID(), RumbleType.kLeftRumble));

    new Trigger(() -> m_Shooter.isAtSpeed(.9))
        .whileTrue(m_feedback.shooterWheelsAtSpeed())
        .and(RobotModeTriggers.teleop())
        .onTrue(shortRumble(m_manipController.getHID(), RumbleType.kBothRumble));

    new Trigger(m_Elevator::elevIsAtPos)
        .and(RobotModeTriggers.teleop())
        .onTrue(shortRumble(m_manipController.getHID(), RumbleType.kBothRumble));

    m_driveController.start().onTrue(Commands.runOnce(() -> m_chassis.getPigeon2().reset()));

    m_driveController
        .rightBumper()
        .whileTrue(
            pickUpNote()
                .deadlineWith(
                    new AlignToNote(m_chassis, () -> new ChassisSpeeds(2, 0, 0))
                        .withName("Auto Note Align")));

    m_driveController
        .leftBumper()
        .whileTrue(
            m_chassis.applyRequest(
                () -> {
                  Translation2d target =
                      DriverStation.getAlliance().orElse(Blue) == Red
                          ? Constants.RED_SPEAKER_POSE
                          : Constants.BLUE_SPEAKER_POSE;
                  Rotation2d angle =
                      target
                          .minus(m_chassis.getState().Pose.getTranslation())
                          .getAngle()
                          .plus(Rotation2d.fromRadians(Math.PI));
                  Logger.recordOutput("Aiming angle", angle);
                  return fieldCentricFacingAngle.withTargetDirection(angle);
                }));

    m_driveController.povDown().whileTrue(m_chassis.applyRequest(() -> brakeMode));

    m_driveController
        .a()
        .or(m_driveController.b())
        .or(m_driveController.x())
        .or(m_driveController.y())
        .whileTrue(
            m_chassis.applyRequest(
                () -> {
                  double xCardinal =
                      (m_driveController.getHID().getYButton() ? 1 : 0)
                          - (m_driveController.getHID().getAButton() ? 1 : 0);
                  double yCardinal =
                      (m_driveController.getHID().getBButton() ? 1 : 0)
                          - (m_driveController.getHID().getXButton() ? 1 : 0);
                  double angle = -Math.atan2(yCardinal, xCardinal);

                  Logger.recordOutput("goalCardinal", angle);
                  return fieldCentricFacingAngle.withTargetDirection(Rotation2d.fromRadians(angle));
                }));

    // Zero the elevator when the robot leaves disabled mode and has not been zeroed
    RobotModeTriggers.disabled()
        .negate()
        .and(m_Elevator::hasNotBeenZeroed)
        .onTrue(m_Elevator.zeroElevator());

    // We don't actually care about the DS being attached, but the robot is 'disabled' by default,
    // so onTrue never trips
    RobotModeTriggers.disabled()
        .and(DriverStation::isDSAttached)
        .onTrue(
            Commands.runOnce(m_feedback::disabledColorPattern)
                .ignoringDisable(true)
                .withName("LEDs Disabled"));

    m_manipController
        .x()
        .whileTrue(
            new VarFeedPrime(
                    m_Shooter,
                    m_Elevator,
                    m_chassis,
                    RobotConstants.SHOOT_POINT,
                    () -> RobotConstants.WRIST_PLOP_ANGLE,
                    1 / RobotConstants.SHOOTER_RPM_TO_MPS,
                    RobotConstants.STRAIGHT_DIST_COEFF)
                .withName("FlatShot"))
        .onFalse(m_Wrist.setToTargetCmd(RobotConstants.WRIST_HIGH_LIM));

    m_manipController
        .b()
        .whileTrue(
            new VarFeedPrime(
                m_Shooter,
                m_Elevator,
                m_chassis,
                RobotConstants.SHOOT_POINT,
                () -> RobotConstants.WRIST_HIGH_LIM,
                1 / RobotConstants.SHOOTER_RPM_TO_MPS,
                RobotConstants.HIGH_DIST_COEFF));

    // Where did the old spinup bind go?
    m_manipController
        .leftTrigger(0.5)
        .whileTrue(
            m_Shooter
                .setSpeedCmd(RobotConstants.SHOOTER_VEL)
                .alongWith(
                    new VarShootPrime(
                        m_Wrist,
                        m_Elevator,
                        m_chassis,
                        RobotConstants.SHOOT_POINT,
                        () -> m_Shooter.getVelocity() * 60,
                        RobotConstants.DISTANCE_RANGE,
                        RobotConstants.HEIGHT_LENGTH_COEFF,
                        RobotConstants.SHOOTER_RPM_TO_MPS,
                        RobotConstants.WRIST_HIGH_LIM))
                .withName("Feed"))
        .onFalse(m_Shooter.setSpeedCmd(0).alongWith(m_Wrist.stow()).withName("Feed End"));

    // Amp position
    m_manipController
        .y()
        .whileTrue(
            m_Wrist
                .setToTargetCmd(Degrees.of(25))
                .alongWith(m_Elevator.setToTarget(16.3))
                .withName("Amp Set-Up"))
        .onFalse(m_Wrist.stow());

    m_manipController.a().whileTrue(m_Elevator.setToTarget(RobotConstants.ELEVATOR_CLIMB_HEIGHT));

    m_manipController.rightBumper().whileTrue(pickUpNote());

    m_manipController.leftBumper().whileTrue(m_feeder.outtake());

    m_manipController.rightTrigger(0.5).whileTrue(m_feeder.shoot());

    m_testController.rightBumper().whileTrue(m_Wrist.setToTargetCmd(Degrees.of(20)));

    m_testController.a().whileTrue(m_Wrist.setToTargetCmd(Degrees.of(30)));

    m_testController.leftBumper().whileTrue(m_Wrist.setToTargetCmd(Degrees.of(40)));

    // Swerve Drive SysId Routines use d-pad
    sysIdController.pov(0).whileTrue(m_chassis.sysIdRotation());
    sysIdController.pov(90).whileTrue(m_chassis.sysIdTranslation());
    sysIdController.pov(180).whileTrue(m_chassis.sysIdSteer());

    sysIdController.a().whileTrue(m_Shooter.sysIdRoutine());
    sysIdController.y().whileTrue(m_Wrist.sysId());
    sysIdController.b().whileTrue(m_Elevator.runSysId());

    RobotModeTriggers.teleop()
        .onTrue(m_Elevator.enableBrakeMode().andThen(m_Wrist.enableBrakeMode()));

    RobotModeTriggers.disabled().onTrue(m_Wrist.enableCoastMode());
    RobotModeTriggers.autonomous().onFalse(m_Shooter.setSpeedCmd(0).ignoringDisable(true));
  }

  public Command pickUpNote() {
    return m_feeder
        .intake()
        .deadlineWith(m_intake.intakeCommand(), m_Wrist.stow())
        .withName("Pick Up Note");
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    if (m_feeder.isNoteQueued()) {
      Translation2d target =
          DriverStation.getAlliance().orElse(Blue) == Red
              ? Constants.RED_SPEAKER_POSE
              : Constants.BLUE_SPEAKER_POSE;
      Rotation2d angle =
          target
              .minus(m_chassis.getState().Pose.getTranslation())
              .getAngle()
              .plus(Rotation2d.fromDegrees(180));
      Logger.recordOutput("Aiming angle", angle);

      return Optional.of(angle);
    } else {
      return Optional.empty();
    }
  }
}
