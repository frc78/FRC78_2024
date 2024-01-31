// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;
  private Pigeon2 pigeon;

  public ChassisSpeeds setChassisSpeed;
  public SwerveModuleState[] setStates;

  public ChassisSpeeds getChassisSpeed;
  public SwerveModuleState[] getStates;
  public SwerveModulePosition[] getPositions;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private PhotonCamera ATCamera;
  private PhotonPoseEstimator photonEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private final Transform3d robotToCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Camera offset

  public Chassis(
      SwerveModule[] modules,
      SwerveDriveKinematics kinematics,
      int pigeonId,
      PhotonCamera ATCamera) {
    // It reads the number of modules from the RobotConstants
    this.modules = modules;
    this.kinematics = kinematics;
    this.ATCamera = ATCamera;

    getChassisSpeed = new ChassisSpeeds();
    getStates = new SwerveModuleState[4];
    getPositions = new SwerveModulePosition[4];

    pigeon = new Pigeon2(pigeonId);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.fromDegrees(getGyroRot()), getPositions(), new Pose2d());
    setChassisSpeed = new ChassisSpeeds();

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.err.println("Failed to load AprilTagFieldLayout");
    }
    photonEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ATCamera, robotToCam);
  }

  public void initializeModules() {
    // This is an example of how we will perform operations on all modules
    for (SwerveModule module : modules) {
      module.initialize();
    }
  }

  public Pose2d getFusedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getGyroRot()), getPositions(), pose);
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(getFusedPose());

    if (estimatedPose.isPresent()) {
      poseEstimator.addVisionMeasurement(
          estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
      Logger.recordOutput("AT Estimate", estimatedPose.get().estimatedPose.toPose2d());
    }
    poseEstimator.update(Rotation2d.fromDegrees(getGyroRot()), getPositions());

    SmartDashboard.putNumber("gyroYaw", getGyroRot());
    Logger.recordOutput("Estimated Pose", poseEstimator.getEstimatedPosition());
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonEstimator.update();
  }

  public double getGyroRot() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public void setGyroRot(double rot) {
    pigeon.setYaw(rot);
  }

  public SwerveModulePosition[] getPositions() {
    for (int i = 0; i < modules.length; i++) {
      getPositions[i] = modules[i].getPosition();
    }
    return getPositions;
  }

  public SwerveModuleState[] getStates() {
    for (int i = 0; i < modules.length; i++) {
      getStates[i] = modules[i].getState();
    }
    return getStates;
  }

  // There is probably a better way to feed this into the AutoBuilder, but this is
  // simpler for now
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public void convertToStates() {
    setStates = kinematics.toSwerveModuleStates(setChassisSpeed);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setChassisSpeed = speeds;
    convertToStates();
    drive();
  }

  public void drive() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(setStates[i]);
      SmartDashboard.putNumber(i + " Rot", setStates[i].angle.getRotations());
    }

    Logger.recordOutput("ModuleSet", setStates);
  }
}
