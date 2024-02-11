// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import com.revrobotics.CANSparkBase;
import frc.robot.classes.Structs.ClosedLoopParameters;
import frc.robot.classes.Structs.FFConstants;

/** Add your docs here. */
public class ModuleConfig {

  public final ClosedLoopParameters driveClosedLoopParameters;
  public final ClosedLoopParameters steerClosedLoopParameters;
  public final FFConstants driveFFConstants;

  public double drivePositionConversionFactor;
  public double driveVelocityConversionFactor;
  public double steerPositionConversionFactor;
  public double steerVelocityConversionFactor;
  public final boolean driveMotorInverted;
  public final boolean steerMotorInverted;
  public final boolean steerEncoderInverted;
  public final double steerEncoderPidMin;
  public final double steerEncoderPidMax;

  public final int driveCurrentLimit;
  public final int steerCurrentLimit;
  public final double nominalVoltage;
  public final CANSparkBase.IdleMode driveIdleMode;
  public final CANSparkBase.IdleMode steerIdleMode;

  public ModuleConfig(
      ClosedLoopParameters driveClosedLoopParameters,
      ClosedLoopParameters steerClosedLoopParameters,
      FFConstants driveFFConstants,
      double drivePositionConversionFactor,
      double driveVelocityConversionFactor,
      double steerPositionConversionFactor,
      double steerVelocityConversionFactor,
      boolean driveMotorInverted,
      boolean steerMotorInverted,
      boolean steerEncoderInverted,
      double steerEncoderPidMin,
      double steerEncoderPidMax,
      int driveCurrentLimit,
      int steerCurrentLimit,
      double nominalVoltage,
      CANSparkBase.IdleMode driveIdleMode,
      CANSparkBase.IdleMode steerIdleMode) {
    this.driveClosedLoopParameters = driveClosedLoopParameters;
    this.steerClosedLoopParameters = steerClosedLoopParameters;
    this.driveFFConstants = driveFFConstants;
    this.drivePositionConversionFactor = drivePositionConversionFactor;
    this.driveVelocityConversionFactor = driveVelocityConversionFactor;
    this.steerPositionConversionFactor = steerPositionConversionFactor;
    this.steerVelocityConversionFactor = steerVelocityConversionFactor;
    this.driveMotorInverted = driveMotorInverted;
    this.steerMotorInverted = steerMotorInverted;
    this.steerEncoderInverted = steerEncoderInverted;
    this.steerEncoderPidMin = steerEncoderPidMin;
    this.steerEncoderPidMax = steerEncoderPidMax;
    this.driveCurrentLimit = driveCurrentLimit;
    this.steerCurrentLimit = steerCurrentLimit;
    this.nominalVoltage = nominalVoltage;
    this.driveIdleMode = driveIdleMode;
    this.steerIdleMode = steerIdleMode;
  }
}
