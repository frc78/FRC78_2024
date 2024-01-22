// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;
  public ChassisSpeeds chassisSpeed;
  public SwerveModuleState[] states;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private Pigeon2 pigeon;

  public Chassis(SwerveModule[] modules, SwerveDriveKinematics kinematics, int pigeonId) {
    // It reads the number of modules from the RobotConstants
    this.modules = modules;
    this.kinematics = kinematics;

    pigeon = new Pigeon2(pigeonId);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.fromDegrees(getGyroRot()), getPositions(), new Pose2d());
    chassisSpeed = new ChassisSpeeds();
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
    poseEstimator.update(Rotation2d.fromDegrees(getGyroRot()), getPositions());

    SmartDashboard.putNumber("gyroYaw", getGyroRot());
    Logger.recordOutput("Estimated Pose", poseEstimator.getEstimatedPosition());
  }

  public double getGyroRot() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public void setGyroRot(double rot) {
    pigeon.setYaw(rot);
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  // There is probably a better way to feed this into the AutoBuilder, but this is simpler for now
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeed;
  }

  public void convertToStates() {
    states = kinematics.toSwerveModuleStates(chassisSpeed);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    chassisSpeed = speeds;
    convertToStates();
    drive();
  }

  public void drive() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
      SmartDashboard.putNumber(i + " Rot", states[i].angle.getRadians());
    }

    Logger.recordOutput("ModuleSet", states);
  }
}
