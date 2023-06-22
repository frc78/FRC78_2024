// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** This is the parent interface for a SwerveModule. */
public interface SwerveModule {
    public void initialize ();
    public void resetToAbsolute ();
    public double getVelocity (); // Get current drive velocity from  (m/s)
    public Rotation2d getRelEncoderPosition (); // Get the relative position of the angle motor's encoder
    public Rotation2d getAbsEncoderPosition (); // Get the absolute position of the magnetic encoder
    public SwerveModuleState getState ();
    public SwerveModulePosition getPosition ();
    public void setVelocity (double velocity); // Set goal drive velocity (m/s)
    public void setRotation (Rotation2d rotation);
    public void setState (SwerveModuleState state);
}
