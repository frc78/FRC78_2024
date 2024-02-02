// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Add your docs here. */
public class PoseEstimator {
  private final Chassis chassis;

  private final SwerveDrivePoseEstimator poseEstimator;
  private PhotonCamera ATCam1;
  private PhotonPoseEstimator photonEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private final Transform3d robotToCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Camera offset

  public PoseEstimator(Chassis chassis, PhotonCamera ATCam1) {
    this.ATCam1 = ATCam1;
    this.chassis = chassis;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            chassis.kinematics,
            Rotation2d.fromDegrees(chassis.getGyroRot()),
            chassis.getPositions(),
            new Pose2d());

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.err.println("Failed to load AprilTagFieldLayout");
    }
    photonEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ATCam1, robotToCam);
  }

  public void update() {
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(getFusedPose());

    if (estimatedPose.isPresent()) {
      poseEstimator.addVisionMeasurement(
          estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
      Logger.recordOutput("AT Estimate", estimatedPose.get().estimatedPose.toPose2d());
    }
    poseEstimator.update(Rotation2d.fromDegrees(chassis.getGyroRot()), chassis.getPositions());

    SmartDashboard.putNumber("gyroYaw", chassis.getGyroRot());
    Logger.recordOutput("Estimated Pose", poseEstimator.getEstimatedPosition());
  }

  public Pose2d getFusedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(chassis.getGyroRot()), chassis.getPositions(), pose);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonEstimator.update();
  }
}
