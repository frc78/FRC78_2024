// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Classes.ModuleConfig;

/** Add your docs here. */
public class NeoModule implements SwerveModule {
    protected ModuleConfig config;
    protected Spark drive;
    protected Spark steer;

    NeoModule(ModuleConfig config) {
        this.config = config;
        drive = new Spark(this.config.driveID);
        steer = new Spark(this.config.steerID);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void resetToAbsolute() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Rotation2d getRelEncoderPosition() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Rotation2d getAbsEncoderPosition() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public SwerveModulePosition getPosition() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setVelocity(double velocity) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        
    }
}
