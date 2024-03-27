package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs.MotionLimits;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AlignToPose extends Command {
  private final Chassis chassis;
  private final PoseEstimator poseEstimator;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final Supplier<Pose2d> goalPoseSupplier;
  private Pose2d goalPose = new Pose2d();

  public AlignToPose(
      Chassis chassis,
      Supplier<Pose2d> goalPoseSupplier,
      PoseEstimator poseEstimator,
      PIDConstants translationPID,
      PIDConstants thetaPID,
      MotionLimits motionLimits) {
    this.chassis = chassis;
    this.poseEstimator = poseEstimator;
    this.goalPoseSupplier = goalPoseSupplier;

    xController =
        new ProfiledPIDController(
            translationPID.kP,
            translationPID.kI,
            translationPID.kD,
            new Constraints(motionLimits.maxSpeed, motionLimits.maxAcceleration));

    xController.setTolerance(.1);
    yController =
        new ProfiledPIDController(
            translationPID.kP,
            translationPID.kI,
            translationPID.kD,
            new Constraints(motionLimits.maxSpeed, motionLimits.maxAcceleration));
    yController.setTolerance(.1);
    thetaController =
        new ProfiledPIDController(
            thetaPID.kP,
            thetaPID.kI,
            thetaPID.kD,
            new Constraints(motionLimits.maxAngularSpeed, motionLimits.maxAngularAcceleration));
    thetaController.setTolerance(3);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    goalPose = goalPoseSupplier.get();
    Pose2d pose = poseEstimator.getFusedPose();
    Transform2d vel = poseEstimator.getEstimatedVel();
    xController.reset(pose.getTranslation().getX(), vel.getX());
    yController.reset(pose.getTranslation().getY(), vel.getY());
    thetaController.reset(pose.getRotation().getRadians(), vel.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = poseEstimator.getFusedPose();

    double xOutput =
        xController.calculate(
            currentPose.getTranslation().getX(), goalPose.getTranslation().getX());
    double yOutput =
        yController.calculate(
            currentPose.getTranslation().getY(), goalPose.getTranslation().getY());
    double thetaOutput =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

    chassis.driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xOutput, yOutput, thetaOutput, currentPose.getRotation()));
    Logger.recordOutput("TargetPose", goalPose);
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    chassis.driveRobotRelative(new ChassisSpeeds());
  }
}
