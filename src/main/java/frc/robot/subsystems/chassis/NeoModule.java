// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.classes.ModuleConfig;

/** Neo implementation of SwerveModule */
public class NeoModule implements SwerveModule {
  protected ModuleConfig config;

  protected CANSparkMax drive;
  protected CANSparkMax steer;

  protected SparkPIDController drivePID;
  protected SparkPIDController steerPID;
  private RelativeEncoder driveEnc;
  private AbsoluteEncoder steerEnc;

  private SwerveModuleState desiredState;

  public NeoModule(ModuleConfig config) {
    this.config = config;

    drive = new CANSparkMax(this.config.driveID, MotorType.kBrushless);
    steer = new CANSparkMax(this.config.steerID, MotorType.kBrushless);

    driveEnc = drive.getEncoder();
    steerEnc = steer.getAbsoluteEncoder(Type.kDutyCycle);
    drivePID = drive.getPIDController();
    steerPID = steer.getPIDController();

    desiredState = new SwerveModuleState();

    initialize();
  }

  /** Initializes the NEO module. This should be called in the constructor of the module class. */
  @Override
  public void initialize() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out
    drive.restoreFactoryDefaults();
    steer.restoreFactoryDefaults();
    drivePID.setFeedbackDevice(driveEnc);
    steerPID.setFeedbackDevice(steerEnc);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEnc.setPositionConversionFactor(config.drivePositionConversionFactor);
    driveEnc.setVelocityConversionFactor(config.driveVelocityConversionFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    steerEnc.setPositionConversionFactor(config.steerPositionConversionFactor);
    steerEnc.setVelocityConversionFactor(config.steerVelocityConversionFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    steerEnc.setInverted(config.steerEncoderInverted);
    steer.setInverted(config.steerMotorInverted);
    drive.setInverted(config.driveMotorInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    steerPID.setPositionPIDWrappingEnabled(true);
    steerPID.setPositionPIDWrappingMinInput(config.steerEncoderPidMin);
    steerPID.setPositionPIDWrappingMaxInput(config.steerEncoderPidMax);

    // Set the PID gains for the driving motor
    drivePID.setP(config.driveClosedLoopParameters.kP);
    drivePID.setI(config.driveClosedLoopParameters.kI);
    drivePID.setD(config.driveClosedLoopParameters.kD);
    drivePID.setFF(config.driveClosedLoopParameters.kF);

    // Set the PID gains for the turning motor
    steerPID.setP(config.steerClosedLoopParameters.kP);
    steerPID.setI(config.steerClosedLoopParameters.kI);
    steerPID.setD(config.steerClosedLoopParameters.kD);
    steerPID.setFF(config.steerClosedLoopParameters.kF);

    drive.setSmartCurrentLimit(config.driveCurrentLimit);
    steer.setSmartCurrentLimit(config.steerCurrentLimit);

    drive.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
    steer.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

    drive.setIdleMode(config.driveIdleMode);
    steer.setIdleMode(config.steerIdleMode);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drive.burnFlash();
    steer.burnFlash();

    desiredState.angle = Rotation2d.fromRotations(steerEnc.getPosition());
    driveEnc.setPosition(0);
  }

  /** Resets the drive encoder to 0 meters */
  @Override
  public void resetEncoders() {
    driveEnc.setPosition(0);
  }

  /**
   * @return The velocity of the drive encoder, in meters per second
   */
  @Override
  public double getDriveVelocity() {
    return driveEnc.getVelocity();
  }

  /**
   * @return The relative position of the drive encoder, in meters
   */
  @Override
  public double getDrivePosition() {
    return driveEnc.getPosition();
  }

  /**
   * @return The absolute position of the drive encoder, as a {@link Rotation2d#Rotation2d
   *     Rotation2d}
   */
  @Override
  public Rotation2d getSteerPosition() {
    return Rotation2d.fromRotations(steerEnc.getPosition());
  }

  /**
   * @return The state of the module as a {@link SwerveModuleState#SwerveModuleState
   *     SwerveModuleState}
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerPosition());
  }

  /**
   * @return The position of the module as a {@link SwerveModulePosition#SwerveModulePosition
   *     SwerveModulePosition}
   */
  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getSteerPosition());
  }

  /**
   * Sets the desired velocity of the module. Should only be used if you want to ONLY set the
   * velocity. If not, then use {@link #setDesiredState(SwerveModuleState)}
   *
   * @param velocity The desired velocity of the module, in meters per second
   */
  @Override
  public void setVelocity(double velocity) {
    drivePID.setReference(velocity, CANSparkMax.ControlType.kVelocity);

    desiredState.speedMetersPerSecond = velocity;
  }

  /**
   * Sets the desired rotation of the module. Should only be used if you want to ONLY set the
   * rotation. If not, then use {@link #setDesiredState(SwerveModuleState)}
   *
   * @param rotation The desired rotation of the module
   */
  @Override
  public void setRotation(Rotation2d rotation) {
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState correctedState =
        SwerveModuleState.optimize(new SwerveModuleState(0, rotation), getSteerPosition());
    steerPID.setReference(correctedState.angle.getRotations(), CANSparkMax.ControlType.kPosition);
    desiredState.angle = rotation;
  }

  /**
   * Sets the desired state of the module. Should be used if you want to set both the velocity and
   * rotation at the same time
   *
   * @param state The desired state of the module
   */
  @Override
  public void setState(SwerveModuleState state) {
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getSteerPosition());

    // Sets the PID goals to the desired states
    drivePID.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kDutyCycle);
    steerPID.setReference(optimizedState.angle.getRotations(), CANSparkMax.ControlType.kPosition);

    desiredState = state;
    SmartDashboard.putNumber(
        config.driveID + " setting rot",
        optimizedState.angle
            .getRotations()); // Changed this to divide by 2 pi and ad o.5 to map the joystick input
    // (-pi to pi) to a zero to 1
    SmartDashboard.putNumber(config.driveID + " getting rot", steerEnc.getPosition() - Math.PI);
    SmartDashboard.putNumber(config.driveID + "getting speed", getDriveVelocity());
    SmartDashboard.putNumber(config.driveID + "setting speed", optimizedState.speedMetersPerSecond);
  }
}
