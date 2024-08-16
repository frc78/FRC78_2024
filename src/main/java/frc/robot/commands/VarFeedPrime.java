// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.ProjectileMotionEquations;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class VarFeedPrime extends Command {
  private Shooter shooter;
  private Elevator elevator;
  private PoseEstimator poseEstimator;
  private Translation2d plopTranslation;
  // private NetworkTableEntry lInput;

  // Translation of where the note exits in the XZ plane (side view)
  private final Translation2d shooterXZTrans;

  private final DoubleSupplier wristAngle;
  private final double MPS_RPM;
  private final double distCoeff;

  /** Creates a new VarShootPrime. */
  public VarFeedPrime(
      Shooter shooter,
      Elevator elevator,
      PoseEstimator poseEstimator,
      Translation2d shooterXZTrans,
      DoubleSupplier wristAngle,
      double MPS_RPM,
      double distCoeff) {
    this.shooter = shooter;
    this.elevator = elevator;
    this.poseEstimator = poseEstimator;
    this.shooterXZTrans = shooterXZTrans;
    this.wristAngle = wristAngle;
    this.MPS_RPM = MPS_RPM;
    this.distCoeff = distCoeff;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    plopTranslation =
        DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get() == Alliance.Red
                ? Constants.RED_PLOP_POSE
                : Constants.BLUE_PLOP_POSE)
            : Constants.BLUE_PLOP_POSE;
  }

  @Override
  public void execute() {
    Pose2d pose = poseEstimator.getFusedPose();

    // Distance and height to speaker
    double distanceToTarget =
        pose.getTranslation().getDistance(plopTranslation) - shooterXZTrans.getX();
    distanceToTarget *= distCoeff;
    // double distanceToTarget = lInput.getDouble(3);

    double heightToTarget = shooterXZTrans.getY() - Units.inchesToMeters(elevator.getElevatorPos());
    // Inverts the height as we are shooting from the robot to the ground, but the calculations are
    // always done from (0, 0) so we use this as our offset
    heightToTarget = -heightToTarget;

    double theta = Math.toRadians(wristAngle.getAsDouble());
    double calcV =
        ProjectileMotionEquations.calculateLaunchVelocity(
            distanceToTarget, heightToTarget, theta);
    double v = Double.isNaN(calcV) ? 0 : calcV;

    Logger.recordOutput("VarFeedPrime theta", theta);
    Logger.recordOutput("VarFeedPrime h", heightToTarget);
    Logger.recordOutput("VarFeedPrime v", v);
    Logger.recordOutput("VarFeedPrime l", distanceToTarget);

    shooter.setSpeed(v * MPS_RPM);
    Logger.recordOutput("VarFeedPrime setV", v * MPS_RPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
  }
}
