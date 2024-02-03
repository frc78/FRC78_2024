// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feed extends SubsystemBase {
  private final TalonFX feedMotor;
  /** Creates a new Feed. */
  public Feed() {
    feedMotor = new TalonFX(14);
  }

  public void setSpeed(double speed){
    feedMotor.set(speed);
  }

  public Command runFeed() {
    return this.startEnd(() -> feedMotor.set(0.5), () -> feedMotor.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
