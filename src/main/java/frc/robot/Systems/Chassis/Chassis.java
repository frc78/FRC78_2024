// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  public SwerveModule module1;

  public Chassis() {
    module1 = new FalconModule();
  }

  @Override
  public void periodic() {
    module1.setVelocity(1);
  }
}
