// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Classes.ModuleConfig;

/** Add your docs here. */
public class NeoModule extends SwerveModule {
    protected ModuleConfig config;
    protected Spark drive;
    protected Spark steer;

    NeoModule(ModuleConfig config) {
        this.config = config;
        drive = new Spark(this.config.driveID);
        steer = new Spark(this.config.steerID);
    }

    // @Override
    public void initialize() {

    }
}
