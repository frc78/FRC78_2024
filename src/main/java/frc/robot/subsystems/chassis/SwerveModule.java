// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

/**
 * This is the parent interface for a SwerveModule. It is used to define the methods that all
 * SwerveModules must implement. Recourse for this can be found in the repository wiki,
 * https://github.com/AquidneckIslandRobotics/78Offseason2023/wiki/Concepts
 */
public interface SwerveModule {
  public void initialize();

  public void resetEncoders();

  /**
   * @return current wheel speed in m/s
   */
  public double getDriveVelocity();

  /**
   * @return current
   */
  public double getDrivePosition(); // Get the relative position of the angle motor's encoder

  /**
   * @return current azimuth position of the module
   */
  public Rotation2d getSteerPosition();

  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();

  /**
   * @param velocity linear velocity of wheel in m/s
   */
  public void setVelocity(double velocity);

  public void setRotation(Rotation2d rotation);

  public void setState(SwerveModuleState state);

  /**
   * Runs the drive motor while keeping the steer angle at 0 degrees.
   *
   * @param voltage voltage to run the motor at
   */
  void openLoopDiffDrive(double voltage);

  /** Logs the motor position, velocity, and voltage data for SysId */
  void logMotor(SysIdRoutineLog log);
}
