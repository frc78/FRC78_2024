import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;

public class AlignToPose extends Command {
  public AlignToPose(Chassis chassis, Supplier<Pose2d> pose, PoseEstimator poseEstimator) {

    addRequirements(chassis);
  }
}
