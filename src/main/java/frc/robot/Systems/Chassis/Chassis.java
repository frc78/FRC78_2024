// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotConstants;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;
  public ChassisSpeeds chassisSpeed;
  public SwerveModuleState[] states;
  
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private Pigeon2 pigeon;

  // StructArrayPublisher<SwerveModuleState> publisher;

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
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getGyroRot() + 90), getPositions(), new Pose2d());
    chassisSpeed = new ChassisSpeeds();

    // publisher = NetworkTableInstance.getDefault()
    // .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    AutoBuilder.configureHolonomic(
                this::getFusedPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                        RobotConstants.MAX_SPEED, // Max module speed, in m/s
                        Constants.ROBOT_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
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

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getGyroRot()), getPositions(), pose);
  }

  @Override
  public void periodic() {
    poseEstimator.update(Rotation2d.fromDegrees(getGyroRot()), getPositions()); //TODO this gyro angle might have to be negated

    SmartDashboard.putNumber("gyroYaw", getGyroRot());
    Logger.recordOutput("Estimated Pose", poseEstimator.getEstimatedPosition());
  }

  public double getGyroRot() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public void setGyroRot(double rot) {
    pigeon.setYaw(rot);
  }

  public SwerveModulePosition[] getPositions () {
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
      SmartDashboard.putNumber(i +" Rot", states[i].angle.getRadians());
    }
  }
}
