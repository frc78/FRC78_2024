// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ROBOT_TYPE;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;

  public Chassis() {
    switch (RobotConstants.ROBOT) {
      case WASP: {
        modules = new SwerveModule[] {
          new FalconModule(RobotConstants.MOD_0_CONFIG),
          new FalconModule(RobotConstants.MOD_1_CONFIG),
          new FalconModule(RobotConstants.MOD_2_CONFIG),
          new FalconModule(RobotConstants.MOD_3_CONFIG)
        };
      }
      case NEO: {
        modules = new SwerveModule[] {
          new NeoModule(RobotConstants.MOD_0_CONFIG),
          new NeoModule(RobotConstants.MOD_1_CONFIG),
          new NeoModule(RobotConstants.MOD_2_CONFIG),
          new NeoModule(RobotConstants.MOD_3_CONFIG)
        };
      }
    }
  }

  public void initializeModules() {
    for (SwerveModule module:modules) {
      module.initialize();
    }
  }

  @Override
  public void periodic() {
    modules[0].setVelocity(1);
  }
}
