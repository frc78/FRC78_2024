// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Classes.ModuleConfig;

/** Falcon implementation of SwerveModule */
public class FalconModule implements SwerveModule {
    protected ModuleConfig config;
    protected TalonFX drive;
    protected TalonFX steer;

    public FalconModule(ModuleConfig config) {
        this.config = config;
        drive = new TalonFX(this.config.driveID);
        steer = new TalonFX(this.config.steerID);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void resetEncoders() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getDriveVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getDrivePosition() {
        // TODO Auto-generated method stub
        return 0.0;
    }

    @Override
    public Rotation2d getSteerPosition() {
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
