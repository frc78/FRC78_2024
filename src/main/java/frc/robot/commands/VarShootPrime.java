// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs.Range;
import frc.robot.classes.Util;
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

  // Translation of where the note exits in the XZ plane (side view)
  private final Translation2d shooterXZTrans;

  private final Range velRange; // Range of velocity from min distance to max distance
  private final Range distRange; // Range of distance from min distance to max distance
  private final double thetaCoeff;
  private final double RPM_MPS;

  /** Creates a new VarShootPrime. */
  public VarShootPrime(
      Wrist wrist,
      Shooter shooter,
      PoseEstimator poseEstimator,
      Translation2d shooterXZTrans,
      Range velRange,
      Range distRange,
      double thetaCoeff,
      double RPM_MPS) {
    this.wrist = wrist;
    this.shooter = shooter;
    this.poseEstimator = poseEstimator;
    this.speakerTranslation = Constants.SPEAKER_TRANSLATION;
    this.shooterXZTrans = shooterXZTrans;
    this.velRange = velRange;
    this.distRange = distRange;
    this.thetaCoeff = thetaCoeff;
    this.RPM_MPS = RPM_MPS;

    addRequirements(wrist, shooter);
  }

  @Override
  public void execute() {
    Pose2d pose = poseEstimator.getFusedPose();

    // Distance and height to speaker
    double h = Constants.SPEAKER_HEIGHT - shooterXZTrans.getY();
    double l = pose.getTranslation().getDistance(speakerTranslation.get()) - shooterXZTrans.getX();
    // Calculate velocity based on lerping within the velocity range based on the distance range
    double v = Util.lerp(Util.clamp(h, distRange) / distRange.getRange(), velRange);
    double theta = calcTheta(Constants.GRAVITY, l, h, v);
    wrist.setToTarget(theta * thetaCoeff);
    shooter.setPIDReferenceBOTH(v / RPM_MPS);
  }

  // Source? It was revealed to me by a wise tree in a dream
  private double calcTheta(double g, double l, double h, double v) {
    double nominator = v * v - Math.sqrt(Math.pow(v, 4) - g * ((g * l * l) + (2 * h * v * v)));
    double denominator = g * l;
    return Math.atan(nominator / denominator);
  }
}
