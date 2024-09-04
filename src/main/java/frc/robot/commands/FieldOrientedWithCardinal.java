// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs;
import frc.robot.subsystems.chassis.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FieldOrientedWithCardinal extends Command {
  private final CommandSwerveDrivetrain chassis;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final Supplier<Pose2d> poseSupplier;
  private final DoubleSupplier direction;

  private final ProfiledPIDController thetaPID;
  private final SimpleMotorFeedforward thetaFF;
  private ChassisSpeeds speeds;
  private double threshold;

  private final SwerveRequest.ApplyChassisSpeeds applyChassisSpeeds =
      new SwerveRequest.ApplyChassisSpeeds();

  /** Set to 180 when on Red */
  private Rotation2d allianceOffset = Rotation2d.fromDegrees(0);

  /** Creates a new FieldOrientedDrive. */
  public FieldOrientedWithCardinal(
      CommandSwerveDrivetrain chassis,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      DoubleSupplier direction,
      PIDConstants cardinalPidConstants,
      Constraints constraints,
      Structs.FFConstants ffConstants,
      double threshold) {
    this.chassis = chassis;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.speeds = speedsSupplier.get();
    this.direction = direction;
    this.threshold = threshold;

    thetaPID =
        new ProfiledPIDController(
            cardinalPidConstants.kP,
            cardinalPidConstants.kI,
            cardinalPidConstants.kD,
            constraints); // TODO tune
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    thetaFF =
        new SimpleMotorFeedforward(ffConstants.kS, ffConstants.kV, ffConstants.kA); // TODO tune
    thetaPID.setTolerance(threshold);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red) {
      allianceOffset = Rotation2d.fromDegrees(180);
    } else {
      allianceOffset = Rotation2d.fromDegrees(0);
    }
    thetaPID.reset(
        poseSupplier.get().getRotation().getRadians(), speedsSupplier.get().omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    speeds = speedsSupplier.get();
    thetaPID.setGoal(direction.getAsDouble());

    double cardinalRotSpeed = thetaPID.calculate(poseSupplier.get().getRotation().getRadians());
    speeds.omegaRadiansPerSecond = cardinalRotSpeed;

    chassis.applyRequest(
        () ->
            applyChassisSpeeds.withSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds, poseSupplier.get().getRotation().plus(allianceOffset))));
    ChassisSpeeds.fromRobotRelativeSpeeds(
        speeds, allianceOffset); // What was this for? forigred moment :skull: TODO
  }

  @Override
  public boolean isFinished() {
    return threshold != 0 && thetaPID.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    chassis.applyRequest(
        () ->
            applyChassisSpeeds.withSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds, poseSupplier.get().getRotation().plus(allianceOffset))));
  }
}
