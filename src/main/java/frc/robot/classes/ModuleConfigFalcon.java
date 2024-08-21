// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Add your docs here. */
public class ModuleConfigFalcon {

  public final CANcoderConfiguration steerEncoderConfig;
  public final TalonFXConfiguration driveMotorConfig;
  public final TalonFXConfiguration steerMotorConfig;
  public final double maxDriveSpeed;
  public final double driveMPS_RPS;

  public ModuleConfigFalcon(
      CANcoderConfiguration steerEncoderConfig,
      TalonFXConfiguration driveMotorConfig,
      TalonFXConfiguration steerMotorConfig,
      double maxDriveSpeed,
      double driveMPS_RPS) {
    this.steerEncoderConfig = steerEncoderConfig;
    this.driveMotorConfig = driveMotorConfig;
    this.steerMotorConfig = steerMotorConfig;
    this.maxDriveSpeed = maxDriveSpeed;
    this.driveMPS_RPS = driveMPS_RPS;
  }
}
