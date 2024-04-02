package frc.robot.subsystems.chassis;

import org.photonvision.PhotonPoseEstimator;

public class NamedPhotonPoseEstimator {
  private final PhotonPoseEstimator poseEstimator;
  private final String name;

  public NamedPhotonPoseEstimator(PhotonPoseEstimator poseEstimator, String name) {
    this.poseEstimator = poseEstimator;
    this.name = name;
  }

  public PhotonPoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public String getName() {
    return name;
  }
}
