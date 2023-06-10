// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Classes.ModuleConfig;

/** Add your docs here. */
public class FalconModule extends SwerveModule {
    protected ModuleConfig config;
    protected TalonFX drive;
    protected TalonFX steer;

    public FalconModule(ModuleConfig config) {
        this.config = config;
        drive = new TalonFX(this.config.driveID);
        steer = new TalonFX(this.config.steerID);
    }

    @Override
    public void setVelocity (double velocity) {
        // Falcon.SetSpeed(velocity);
    }
}
