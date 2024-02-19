// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;

public class VarShootPrime extends Command {
  private Wrist wrist;
  private Shooter shooter;
  private PoseEstimator poseEstimator;
  private Supplier<Translation2d> speakerTranslation;
  private Translation2d shooterXZTrans;

  /** Creates a new VarShootPrime. */
  public VarShootPrime(
      Wrist wrist, Shooter shooter, PoseEstimator poseEstimator, Translation2d shooterXZTrans) {
    this.wrist = wrist;
    this.shooter = shooter;
    this.poseEstimator = poseEstimator;
    this.speakerTranslation = Constants.SPEAKER_TRANSLATION;
    this.shooterXZTrans = shooterXZTrans;

    addRequirements(wrist, shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d pose = poseEstimator.getFusedPose();

    double h = Constants.SPEAKER_HEIGHT - shooterXZTrans.getY();
    double l =
        poseEstimator.getFusedPose().getTranslation().getDistance(speakerTranslation.get())
            - shooterXZTrans.getX();
    double v = 100; // TODO
    double theta = calcTheta(Constants.GRAVITY, l, h, v);
    wrist.setAngle(theta);
    shooter.setSpeed(v);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  // Source? It was revealed to me by a wise tree in a dream
  private double calcTheta(double g, double l, double h, double v) {
    double nominator = v * v - Math.sqrt(Math.pow(v, 4) - g * ((g * l * l) + (2 * h * v * v)));
    double denominator = g * l;
    return Math.atan(nominator / denominator);
  }
}
