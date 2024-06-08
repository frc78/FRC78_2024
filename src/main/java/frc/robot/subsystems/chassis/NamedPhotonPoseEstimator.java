package frc.robot.subsystems.chassis;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class NamedPhotonPoseEstimator extends PhotonPoseEstimator {
  private final String name;

  public NamedPhotonPoseEstimator(
      AprilTagFieldLayout layout,
      PoseStrategy strategy,
      PhotonCamera camera,
      Transform3d robotToCamera,
      String name) {
    super(layout, strategy, camera, robotToCamera);
    this.name = name;
  }

  public String getName() {
    return name;
  }
}
