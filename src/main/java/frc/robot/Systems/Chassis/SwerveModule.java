// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

/** Add your docs here. */
public interface SwerveModule {
    public void setVelocity (double velocity);
    public void SetRotation (double rotation);
    public void initialize ();
}
