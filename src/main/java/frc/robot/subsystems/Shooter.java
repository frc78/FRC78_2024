// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX shooter1;
  private final TalonFX shooter2;

  public Shooter(int shooter1ID, int shooter2ID) {
    shooter1 = new TalonFX(shooter1ID);
    shooter2 = new TalonFX(shooter2ID);
  }

  public Command setShooter(double speed) {
    return new InstantCommand(() -> {
      shooter1.set(speed);
      shooter2.set(speed);
    });
  }
}
