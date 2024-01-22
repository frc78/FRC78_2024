// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Classes;

/** Add your docs here. */
public class ModuleConfig {
    public final int driveID;
    public final int steerID;
    public final int encID;
    public final double offset;

    public ModuleConfig(int driveID, int steerID, int encID, double offset) {
        this.driveID = driveID;
        this.steerID = steerID;
        this.encID = encID;
        this.offset = offset;
    }
}
