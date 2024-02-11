package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs.FFConstants;
import frc.robot.classes.Structs.MotionLimits;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;

public class AlignToPose extends Command {
  private Chassis chassis;
  private PoseEstimator poseEstimator;
  private Supplier<Pose2d> targetPose;

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  public AlignToPose(
      Chassis chassis,
      Supplier<Pose2d> targetPose,
      PoseEstimator poseEstimator,
      PIDConstants translationPID,
      PIDConstants thetaPID,
      FFConstants translationFF,
      FFConstants thetaFF,
      MotionLimits motionLimits) {
    this.chassis = chassis;
    this.poseEstimator = poseEstimator;
    this.targetPose = targetPose;

    xController =
        new ProfiledPIDController(
            translationPID.kP,
            translationPID.kI,
            translationPID.kD,
            new Constraints(motionLimits.maxSpeed, motionLimits.maxAcceleration));
    yController =
        new ProfiledPIDController(
            translationPID.kP,
            translationPID.kI,
            translationPID.kD,
            new Constraints(motionLimits.maxSpeed, motionLimits.maxAcceleration));
    thetaController =
        new ProfiledPIDController(
            thetaPID.kP,
            thetaPID.kI,
            thetaPID.kD,
            new Constraints(motionLimits.maxAngularSpeed, motionLimits.maxAngularAcceleration));

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d getPose = targetPose.get();
    Pose2d currentPose = poseEstimator.getFusedPose();

    var xOutput =
        xController.calculate(
            currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
    var yOutput =
        yController.calculate(
            currentPose.getTranslation().getY(), targetPose.getTranslation().getY());
    var thetaOutput =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    chassis.drive(xOutput, yOutput, thetaOutput);
  }
}
