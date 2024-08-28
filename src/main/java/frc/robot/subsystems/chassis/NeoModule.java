// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.classes.ModuleConfig;
import frc.robot.classes.Structs.FFConstants;
import org.littletonrobotics.junction.Logger;

/** Neo implementation of SwerveModule */
public class NeoModule implements SwerveModule {

  protected ModuleConfig config;

  protected int driveID;
  protected int steerID;

  protected SimpleMotorFeedforward driveFF;

  private SwerveModuleState settingState;
  private SwerveModuleState realState;
  private SwerveModuleState optimizedState;

  public NeoModule(int driveID, int steerID, ModuleConfig config, FFConstants ffConstants) {
    this.config = config;

    this.driveID = driveID;
    this.steerID = steerID;

    driveFF = new SimpleMotorFeedforward(ffConstants.kS, ffConstants.kV, ffConstants.kA);

    settingState = new SwerveModuleState();
    realState = new SwerveModuleState();
    optimizedState = new SwerveModuleState();

    initialize();
  }

  /** Initializes the NEO module. This should be called in the constructor of the module class. */
  @Override
  public void initialize() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.

    // Set the PID gains for the driving motor

    // Set the PID gains for the turning motor
  }

  @Override
  public void setBrake(Boolean y) {}

  /**
   * @return The velocity of the drive encoder, in meters per second
   */
  @Override
  public double getDriveVelocity() {
    return 0;
  }

  /**
   * @return The relative position of the drive encoder, in meters
   */
  @Override
  public double getDrivePosition() {
    return 0;
  }

  /**
   * @return The absolute position of the drive encoder, as a {@link Rotation2d#Rotation2d
   *     Rotation2d}
   */
  @Override
  public Rotation2d getSteerPosition() {
    return Rotation2d.fromRotations(0);
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

  @Override
  public SwerveModuleState getOptimizedState() {
    return optimizedState;
  }

  @Override
  public SwerveModuleState getRealState() {
    return realState;
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
    optimizedState = SwerveModuleState.optimize(state, getSteerPosition());
    double speedModifier =
        Math.abs(Math.cos(((optimizedState.angle.getRadians()) - getSteerPosition().getRadians())));
    optimizedState.speedMetersPerSecond *= speedModifier;
    Logger.recordOutput("Swerve speed modifier", speedModifier);

    // Sets the PID goals to the desired states
    settingState = state;
    realState.speedMetersPerSecond = getDriveVelocity();
    realState.angle = getSteerPosition();
    // (-pi to pi) to a zero to 1
    // Logger.recordOutput(driveID + " drive meters", driveEnc.getPosition());
    Logger.recordOutput(
        driveID + "steer err",
        optimizedState.angle.getRotations() - realState.angle.getRotations());
  }

  public void openLoopDiffDrive(double voltage) {}

  /*
   * Mutate these each time we log so that we aren't creating objects constantly
   */
  private final MutableMeasure<Voltage> mutableAppliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Distance> mutableDistance = MutableMeasure.mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> mutableVelocity =
      MutableMeasure.mutable(MetersPerSecond.of(0));

  public void logMotor(SysIdRoutineLog log) {
    log.motor("motor#" + driveID)
        // Log voltage
        .voltage(
            /* getAppliedOutput returns the duty cycle which is from [-1, +1].
            We multiply this by the voltage going into the spark max,
            called the bus voltage to receive the output voltage */
            mutableAppliedVoltage.mut_replace(0, Volts))
        // the drive encoder has the necessary position and velocity conversion factors already set
        .linearVelocity(mutableVelocity.mut_replace(0, MetersPerSecond))
        .linearPosition(mutableDistance.mut_replace(0, Meters));
  }

  @Override
  public void enableBrakeMode() {}

  @Override
  public void enableCoastMode() {}
}
