// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotConstants;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;
  public ChassisSpeeds chassisSpeed;
  public SwerveModuleState[] states;
  
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private Pigeon2 pigeon;

  public Chassis() {
    // It reads the number of modules from the RobotConstants
    modules = new SwerveModule[RobotConstants.MOD_CONFIGS.length];
    for (int i = 0; i < RobotConstants.MOD_CONFIGS.length; i++) {
      switch (RobotConstants.ROBOT) {
        case WASP: {
          modules[i] = new FalconModule(RobotConstants.MOD_CONFIGS[i]);
          break;
        }
        case NEO: {
          modules[i] = new NeoModule(RobotConstants.MOD_CONFIGS[i]);
          break;
        }
      }
    }

    pigeon = new Pigeon2(RobotConstants.PIGEON_ID);
    kinematics = Constants.SWERVE_KINEMATICS;
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getGyroRot()), getPositions(), new Pose2d());
  }

  public void initializeModules() {
    // This is an example of how we will perform operations on all modules
    for (SwerveModule module:modules) {
      module.initialize();
    }
  }

  public Pose2d getFusedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    poseEstimator.update(Rotation2d.fromDegrees(getGyroRot()), getPositions());
  }

  public double getGyroRot() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public SwerveModulePosition[] getPositions () {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public void convertToStates() {
     states = kinematics.toSwerveModuleStates(chassisSpeed);
  }

  public void drive() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
      SmartDashboard.putNumber(i +" Rot", states[i].angle.getRadians());
    }
  }
}
