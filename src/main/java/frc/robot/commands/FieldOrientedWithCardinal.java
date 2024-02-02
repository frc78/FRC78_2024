// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class FieldOrientedWithCardinal extends Command {
  private final Chassis chassis;
  private final ChassisSpeeds speeds;
  private final PoseEstimator poseEstimator;
  private final BooleanSupplier upSupplier, rightSupplier, downSupplier, leftSupplier;

  private final ProfiledPIDController thetaPID;
  private final SimpleMotorFeedforward thetaFF;

  /** Creates a new FieldOrientedDrive. */
  public FieldOrientedWithCardinal(
      Chassis chassis,
      PoseEstimator poseEstimator,
      XboxController controller,
      ChassisSpeeds speeds,
      PIDConstants cardinalPidConstants,
      Constraints constraints,
      Structs.FFConstants ffConstants) {
    this.chassis = chassis;
    this.poseEstimator = poseEstimator;
    this.speeds = speeds;

    this.upSupplier = () -> controller.getYButton();
    this.rightSupplier = () -> controller.getBButton();
    this.downSupplier = () -> controller.getAButton();
    this.leftSupplier = () -> controller.getXButton();

    thetaPID =
        new ProfiledPIDController(
            cardinalPidConstants.kP,
            cardinalPidConstants.kI,
            cardinalPidConstants.kD,
            constraints); // TODO tune
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    thetaFF =
        new SimpleMotorFeedforward(ffConstants.kS, ffConstants.kV, ffConstants.kA); // TODO tune

    addRequirements(chassis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xCardinal = (upSupplier.getAsBoolean() ? 1 : 0) - (downSupplier.getAsBoolean() ? 1 : 0);
    double yCardinal =
        (rightSupplier.getAsBoolean() ? 1 : 0) - (leftSupplier.getAsBoolean() ? 1 : 0);
    double dir = -Math.atan2(yCardinal, xCardinal);
    dir = dir < 0 ? dir + 2 * Math.PI : dir; // TODO check if needed

    Logger.recordOutput("goalCardinal", dir);

    thetaPID.setGoal(dir);

    double cardinalRotSpeed =
        upSupplier.getAsBoolean()
                || rightSupplier.getAsBoolean()
                || downSupplier.getAsBoolean()
                || leftSupplier.getAsBoolean()
            ? thetaPID.calculate(poseEstimator.getFusedPose().getRotation().getRadians())
            : 0;
    speeds.omegaRadiansPerSecond += cardinalRotSpeed;

    chassis.driveRobotRelative(
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, poseEstimator.getFusedPose().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.driveRobotRelative(new ChassisSpeeds());
  }
}
