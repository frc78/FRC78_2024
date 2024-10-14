// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * This class is responsible for periodically retrieving vision measurements from networktables and
 * updating the drivetrain with them.
 */
public class Vision {
  private final CommandSwerveDrivetrain chassis;
  private static final String STERN_CAM_NAME = "SternCam";
  private static final String STARBOARD_CAM_NAME = "StarboardCam";
  private static final String PORT_CAM_NAME = "PortCam";

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

  private final List<NamedPhotonPoseEstimator> poseEstimators;
  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  public Vision(CommandSwerveDrivetrain chassis) {
    this.chassis = chassis;

    NamedPhotonPoseEstimator sternCam =
        new NamedPhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera(STERN_CAM_NAME),
            new Transform3d(
                new Translation3d(-4.5, 0, 17.902).times(Units.inchesToMeters(1)),
                new Rotation3d(Math.PI, Math.toRadians(-30), Math.PI)),
            STERN_CAM_NAME);
    NamedPhotonPoseEstimator starboardCam =
        new NamedPhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera(STARBOARD_CAM_NAME),
            new Transform3d(
                new Translation3d(4.465, -10.205, 21.274).times(Units.inchesToMeters(1)),
                new Rotation3d(Math.PI, Math.toRadians(-25), Math.toRadians(-30))),
            STARBOARD_CAM_NAME);
    NamedPhotonPoseEstimator portCam =
        new NamedPhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera(PORT_CAM_NAME),
            new Transform3d(
                new Translation3d(4.465, 10.205, 21.274).times(Units.inchesToMeters(1)),
                new Rotation3d(0, Math.toRadians(-25), Math.toRadians(30))),
            PORT_CAM_NAME);

    // Uncomment when orange pi is fixed
    //    poseEstimators = List.of(sternCam, starboardCam, portCam);
    poseEstimators = List.of(sternCam);
  }

  /**
   * Update the drivetrain with the latest vision measurements.
   *
   * <p>This method should be called periodically
   */
  public void updateVisionMeasurements() {
    for (NamedPhotonPoseEstimator poseEstimator : poseEstimators) {
      Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update();

      if (estimatedPoseOptional.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
        Pose2d estPose = estimatedRobotPose.estimatedPose.toPose2d();
        // Change our trust in the measurement based on the tags we can see
        Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose, estimatedRobotPose.targetsUsed);

        chassis.addVisionMeasurement(estPose, estimatedRobotPose.timestampSeconds, estStdDevs);

        Logger.recordOutput(poseEstimator.getName() + "Estimate", estPose);
      }
    }
  }

  private Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targetsUsed) {
    var estStdDevs = SINGLE_TAG_STD_DEVS;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targetsUsed) {
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) {
      estStdDevs = MULTI_TAG_STD_DEVS;
    }
    // Increase std devs based on (average) distance
    return estStdDevs.times(1 + (avgDist * avgDist / 30));
  }
}
