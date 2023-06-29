// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;

  // private double[] inputs;
  // private double[] weights;

  public Chassis() {
    modules = new SwerveModule[RobotConstants.MOD_CONFIGS.length];
    for (int i = 0; i < RobotConstants.MOD_CONFIGS.length; i++) {
      switch (RobotConstants.ROBOT) {
        case WASP: {
          modules[i] = new FalconModule(RobotConstants.MOD_CONFIGS[i]);
          break;
        }
        case NEO: {
          modules[i] = new NeoModule(RobotConstants.MOD_CONFIGS[i]);
          break;
        }
      }
    }
  }

  public void initializeModules() {
    // This is an example of how we will perform operations on all modules
    for (SwerveModule module:modules) {
      module.initialize();
    }
  }

  // public void addInput(double input, double weight) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
